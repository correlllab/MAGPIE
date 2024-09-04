'''
@file label_owlv2.py
@brief OWLv2 implementation of label.py
'''
import sys
sys.path.append("../../")
import torch
import numpy as np
from magpie.perception.label import Label
from magpie.perception.object import Object
from transformers import Owlv2Processor, Owlv2ForObjectDetection
from transformers.utils.constants import OPENAI_CLIP_MEAN, OPENAI_CLIP_STD
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

class LabelOWLv2(Label):
    def __init__(self, topk=3, score_threshold=0.005, pth="google/owlv2-base-patch16-ensemble"):
        '''
        @param camera camera object, expects realsense_wrapper
        '''
        super().__init__()
        self.processor = Owlv2Processor.from_pretrained(pth)
        self.model = Owlv2ForObjectDetection.from_pretrained(pth)
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk

    def box_coordinates(self, box):
        '''
        @param box params [x_center, y_center, width, height]
        @return list of box corners [x_min, y_min, x_max, y_max]
        '''
        return box

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
        ax.imshow(input_image)
        ax.set_axis_off()


        idx = 0
        if topk:
            scores = self.sorted_scores[:self.TOP_K]
            boxes  = self.sorted_boxes_coords[:self.TOP_K]
            labels = self.sorted_labels[:self.TOP_K] # oops
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD and not topk:
                continue
            x1, y1, x2, y2 = box
            width = x2 - x1
            height = y2 - y1
            rect = patches.Rectangle((x1, y1), width, height, linewidth=2, edgecolor='r', facecolor='none')
            ax.add_patch(rect)
            ax.text(x1, y1, f'{text_queries[label]} ({idx}): {score:.3f}', color='red', verticalalignment='top', 
                    bbox={'facecolor': 'white', 'alpha': 0.5, 'pad': 1})

            idx += 1
        
        fig.canvas.draw()
        predicted_image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        predicted_image = predicted_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        self.preds_plot = predicted_image
        if not show_plot: plt.close(fig)  # Close the figure to prevent displaying it

    def get_preds(self, outputs, target_sizes):
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        self.results = self.processor.post_process_object_detection(outputs=outputs, target_sizes=target_sizes, threshold=self.SCORE_THRESHOLD)
        boxes  = self.results[0]['boxes'].cpu().detach().numpy()
        pboxes = self.results[0]['boxes'].cpu().detach().numpy()
        scores = self.results[0]['scores'].cpu().detach().numpy()
        labels = self.results[0]['labels'].cpu().detach().numpy()
        # sort labels by score, high to low
        sorted_indices = np.argsort(scores)[::-1]

        # store member variables
        # cut off score indices below threshold
        self.sorted_indices = sorted_indices[scores[sorted_indices] > self.SCORE_THRESHOLD]
        self.sorted_scores = scores[self.sorted_indices]
        self.sorted_labels = labels[self.sorted_indices]
        self.sorted_text_labels = np.array([self.queries[label] for label in labels[self.sorted_indices]])
        self.sorted_boxes = boxes[self.sorted_indices]
        self.sorted_boxes_coords = boxes[self.sorted_indices]
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

        target_sizes = torch.Tensor([[max(self.W,self.H), max(self.W,self.H)]])
        
        self.queries = abbrev_labels
        scores, labels, boxes, pboxes = self.get_preds(outputs, target_sizes)
        self.plot_predictions(img, abbrev_labels, scores, boxes, labels, topk=topk, show_plot=plot)
        bboxes, uboxes = self.get_boxes(input_image, abbrev_labels, scores, boxes, labels)
        self.boxes = bboxes
        self.labels = np.array([i[1] for i in uboxes])
        return bboxes, uboxes

