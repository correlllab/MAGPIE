'''
@file label_owlvit.py
@brief OWL-ViT implementation of label.py
'''
import sys
sys.path.append("../../")
import torch
import numpy as np
from magpie.perception.label import Label
from magpie.perception.object import Object
from transformers import OwlViTProcessor, OwlViTForObjectDetection
import matplotlib.pyplot as plt

class LabelOWLViT(Label):
    def __init__(self, topk=4, score_threshold=0.005, pth="google/owlvit-base-patch32"):
        '''
        @param camera camera object, expects realsense_wrapper
        '''
        super().__init__()
        self.processor = OwlViTProcessor.from_pretrained(pth)
        self.model = OwlViTForObjectDetection.from_pretrained(pth)
        self.dims = None
        self.H = None
        self.W = None
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk
        self.preds_plot = None
        self.queries = None
        self.sorted_indices = None
        self.sorted_labels = None
        self.sorted_text_labels = None
        self.sorted_scores = None
        self.sorted_boxes = None
        self.sorted_boxes_coords = None
        self.sorted_labeled_boxes = None
        self.sorted_labeled_boxes_coords = None
        self.sorted = None
        self.boxes = None

    def box_coordinates(self, box):
        '''
        @param box params [x_center, y_center, width, height]
        @return list of box corners [x_min, y_min, x_max, y_max]
        '''
        cx, cy, w, h = box
        x0 = (cx - w/2) * self.W
        y0 = (cy - h/2) * self.H
        x1 = (cx + w/2) * self.W
        y1 = (cy + h/2) * self.H
        coordinates = [x0, y0, x1, y1]
        return coordinates

    def get_boxes(self, input_image, text_queries, scores, boxes, labels):
        pboxes = []
        uboxes = []
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD:
                continue
            pbox = self.box_coordinates(box)
            pboxes.append((pbox, text_queries[label]))
            uboxes.append((box, text_queries[label]))
        return pboxes, uboxes
    
    def plot_predictions(self, input_image, text_queries, scores, boxes, labels, topk=False, show_plot=True):
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()
        
        idx = 0
        if topk:
            scores = self.sorted_scores[:self.TOP_K]
            boxes  = self.sorted_boxes[:self.TOP_K]
            labels = self.sorted_labels[:self.TOP_K] # oops
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD and not topk:
                continue
            cx, cy, w, h = box
            ax.plot([cx-w/2, cx+w/2, cx+w/2, cx-w/2, cx-w/2],
                    [cy-h/2, cy-h/2, cy+h/2, cy+h/2, cy-h/2], "r")
            ax.text(
                cx - w / 2,
                cy + h / 2 + 0.015,
                f"{text_queries[label]} ({idx}): {score:1.2f}",
                ha="left",
                va="top",
                color="red",
                bbox={
                    "facecolor": "white",
                    "edgecolor": "red",
                    "boxstyle": "square,pad=.3"
                })
            idx += 1
        
        fig.canvas.draw()
        predicted_image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        predicted_image = predicted_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        self.preds_plot = predicted_image
        if not show_plot: plt.close(fig)  # Close the figure to prevent displaying it


    def reset_prediction_state( self ):
        self.preds_plot = None
        self.queries = None
        self.sorted_indices = None
        self.sorted_labels = None
        self.sorted_text_labels = None
        self.sorted_scores = None
        self.sorted_boxes = None
        self.sorted_boxes_coords = None
        self.sorted_labeled_boxes = None
        self.sorted_labeled_boxes_coords = None
        self.sorted = None
        self.boxes = None


    def get_preds(self, outputs, target_sizes):

        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        # boxes = outputs["pred_boxes"][0].cpu().detach().numpy()
        boxes = outputs["pred_boxes"][0].cpu().detach().numpy()
        pboxes = self.processor.post_process_object_detection(outputs=outputs, target_sizes=target_sizes, threshold=self.SCORE_THRESHOLD)[0]['boxes']
        # sort labels by score, high to low
        sorted_indices = np.argsort(scores)[::-1]

        # store member variables
        # cut off score indices below threshold
        self.sorted_indices = sorted_indices[scores[sorted_indices] > self.SCORE_THRESHOLD]
        self.sorted_scores = scores[self.sorted_indices]
        self.sorted_labels = labels[self.sorted_indices]
        self.sorted_text_labels = np.array([self.queries[label] for label in labels[self.sorted_indices]])
        self.sorted_boxes = boxes[self.sorted_indices]
        self.sorted_boxes_coords = np.array([self.box_coordinates(box) for box in boxes[self.sorted_indices]])
        self.sorted_labeled_boxes = list(zip(self.sorted_boxes, self.sorted_labels))
        self.sorted_labeled_boxes_coords = list(zip(self.sorted_boxes_coords, self.sorted_labels))
        self.sorted = list(zip(self.sorted_scores, self.sorted_labels, self.sorted_indices, self.sorted_boxes))
        
        return scores, labels, boxes, pboxes

    def label(self, input_image, input_labels, abbrev_labels, topk=False, plot=False):
        '''
        @param input_labels list of input labels
        @param input_image np.array image to label
        @return pboxes list of predicted boxes
        @return uboxes list of unnormalized boxes
        '''
        img = np.asarray(input_image)
        img_tensor = torch.tensor(img, dtype=torch.float32)
        inputs = self.processor(input_labels, images=img_tensor, padding=True, return_tensors="pt")

        outputs = self.model(**inputs)
        self.dims = img.shape[:2][::-1] # TODO: check if this is correct
        self.W = self.dims[0]
        self.H = self.dims[1]
        target_sizes = torch.Tensor([self.dims])
        self.queries = abbrev_labels
        scores, labels, boxes, pboxes = self.get_preds(outputs, target_sizes)
        image_plt = img.astype(np.float32) / 255.0
        self.plot_predictions(image_plt, abbrev_labels, scores, boxes, labels, topk=topk, show_plot=plot)
        bboxes, uboxes = self.get_boxes(input_image, abbrev_labels, scores, boxes, labels)
        self.boxes = bboxes
        self.labels = np.array([i[1] for i in uboxes])
        return bboxes, uboxes, scores, labels

