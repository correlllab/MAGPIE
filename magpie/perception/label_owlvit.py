'''
@file label_owlvit.py
@brief OWL-ViT implementation of label.py
'''
import sys
sys.path.append("../../")
import torch
import numpy as np
from magpie.perception.label import Label
from transformers import OwlViTProcessor, OwlViTForObjectDetection
import matplotlib.pyplot as plt

class LabelOWLViT(Label):
    def __init__(self, pth="google/owlvit-base-patch32"):
        '''
        @param camera camera object, expects realsense_wrapper
        '''
        super().__init__()
        self.processor = OwlViTProcessor.from_pretrained(pth)
        self.model = OwlViTForObjectDetection.from_pretrained(pth)
        self.dims = None
        self.H = None
        self.W = None
        self.SCORE_THRESHOLD = 0.01

    def get_boxes(self, input_image, text_queries, scores, boxes, labels):
        pboxes = []
        uboxes = []
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD:
                continue
            cx, cy, w, h = box
            x0 = (cx - w/2) * self.W
            y0 = (cy - h/2) * self.H
            x1 = (cx + w/2) * self.W
            y1 = (cy + h/2) * self.H
            # x0 = (cx - w/2) * 1280
            # y0 = (cy - h/2) * 720
            # x1 = (cx + w/2) * 1280
            # y1 = (cy + h/2) * 720
            pbox = [x0, y0, x1, y1]
            pboxes.append((pbox, text_queries[label]))
            uboxes.append((box, text_queries[label]))
        return pboxes, uboxes
    
    def plot_predictions(self, input_image, text_queries, scores, boxes, labels):
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()
        
        idx = 0
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD:
                continue
            cx, cy, w, h = box
            x0 = (cx - w/2) * 1280
            y0 = (cy - h/2) * 720
            x1 = (cx + w/2) * 1280
            y1 = (cy + h/2) * 720
            pbox = [x0, y0, x1, y1]
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

    def get_preds(self, outputs, target_sizes):
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        # boxes = outputs["pred_boxes"][0].cpu().detach().numpy()
        boxes = outputs["pred_boxes"][0].cpu().detach().numpy()
        pboxes = self.processor.post_process_object_detection(outputs=outputs, target_sizes=target_sizes, threshold=self.SCORE_THRESHOLD)[0]['boxes']
        return scores, labels, boxes, pboxes

    def label(self, input_image, input_labels, abbrev_labels, plot=False):
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
        scores, labels, boxes, pboxes = self.get_preds(outputs, target_sizes)
        image_plt = img.astype(np.float32) / 255.0
        if plot:
            self.plot_predictions(image_plt, abbrev_labels, scores, boxes, labels)
        bboxes, uboxes = self.get_boxes(input_image, abbrev_labels, scores, boxes, labels)
        self.boxes = bboxes
        self.labels = np.array([i[1] for i in uboxes])
        return bboxes, uboxes

    def set_threshold(self, threshold):
        self.SCORE_THRESHOLD = threshold
