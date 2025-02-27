'''
@file label_owlvit.py
@brief OWL-ViT implementation of label.py
'''
import sys
sys.path.append("../")
import torch
import numpy as np
from magpie_perception.label import Label
from transformers import OwlViTProcessor, OwlViTForObjectDetection
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

class LabelOWLViT(Label):
    def __init__(self, topk=3, score_threshold=0.005):
        '''
        @param camera camera object, expects realsense_wrapper
        '''
        super().__init__()
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk

    def init(self, topk=3, score_threshold=0.005, pth="google/owlvit-base-patch32"):
        self.processor = OwlViTProcessor.from_pretrained(pth)
        self.model = OwlViTForObjectDetection.from_pretrained(pth)
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk

    def xywh_to_x1y1x2y2(self, box):
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
    
    def get_preds(self, outputs, target_sizes):
        logits = torch.max(outputs["logits"][0], dim=-1)
        self.scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        self.labels = logits.indices.cpu().detach().numpy()
        self.boxes = outputs["pred_boxes"][0].cpu().detach().numpy()
        self.results = self.processor.post_process_object_detection(outputs=outputs, target_sizes=target_sizes, threshold=self.SCORE_THRESHOLD)
        pboxes = self.results[0]['boxes']
        # sort labels by score, high to low
        sorted_indices = np.argsort(self.scores)[::-1]

        # store member variables
        # cut off score indices below threshold
        self.sorted_indices = sorted_indices[self.scores[sorted_indices] > self.SCORE_THRESHOLD]
        self.sorted_scores = self.scores[self.sorted_indices]
        self.sorted_labels = self.labels[self.sorted_indices]
        self.sorted_text_labels = np.array([self.queries[label] for label in self.labels[self.sorted_indices]])
        self.sorted_boxes = self.boxes[self.sorted_indices]
        self.sorted_boxes_coords = np.array([self.xywh_to_x1y1x2y2(box) for box in self.boxes[self.sorted_indices]])
        self.sorted_labeled_boxes = list(zip(self.sorted_boxes, self.sorted_labels))
        self.sorted_labeled_boxes_coords = list(zip(self.sorted_boxes_coords, self.sorted_labels))
        self.sorted = list(zip(self.sorted_scores, self.sorted_labels, self.sorted_indices, self.sorted_boxes))
        
        return self.scores, self.labels, self.boxes, pboxes

    def label(self, input_image, input_labels, abbrev_labels, topk=False, plot=False):
        '''
        @param input_labels list of input labels
        @param input_image np.array image to label
        @return pboxes list of predicted boxes
        @return uboxes list of unnormalized boxes
        '''
        self.image = img = np.asarray(input_image)
        img_tensor = torch.tensor(img, dtype=torch.float32)
        inputs = self.processor(input_labels, images=img_tensor, padding=True, return_tensors="pt")
        outputs = self.model(**inputs)
        self.dims = img.shape[:2][::-1] # TODO: check if this is correct
        self.W = self.dims[0]
        self.H = self.dims[1]
        target_sizes = torch.Tensor([self.dims])
        self.queries = abbrev_labels
        scores, labels, boxes, pboxes = self.get_preds(outputs, target_sizes)
        # image_plt = img.astype(np.float32) / 255.0
        self.plot_predictions(topk=topk, show_plot=plot)
        return self.results, self.sorted_boxes_coords, self.sorted_scores, self.sorted_labels

