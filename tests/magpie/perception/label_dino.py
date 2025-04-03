'''
@file label_dinopy
@brief DINO implementation of label.py
NOTES:
Query structure is single string, with elements separated by periods.
Uses `post_process_grounded_object_detection`
Different call structure to `processor` instantiation

Available models:
IDEA-Research/grounding-dino-tiny
IDEA-Research/grounding-dino-base
'''
'''
@file label_dinopy
@brief DINO implementation of label.py
NOTES:
Query structure is single string, with elements separated by periods.
Uses `post_process_grounded_object_detection`
Different call structure to `processor` instantiation

Available models:
IDEA-Research/grounding-dino-tiny
IDEA-Research/grounding-dino-base
'''
import sys
sys.path.append("../")
import torch
import numpy as np
from magpie_perception.label import Label
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

class LabelDINO(Label):
    def __init__(self, topk=3, score_threshold=0.005):
        '''
        @param camera camera object, expects realsense_wrapper
        '''
        super().__init__()
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk

    def init(self, topk=3, score_threshold=0.005, pth="IDEA-Research/grounding-dino-tiny"):
        self.processor = AutoProcessor.from_pretrained(pth)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(pth).to(self.device)
        self.SCORE_THRESHOLD = score_threshold
        self.TOP_K = topk

    def get_boxes(self, input_image, text_queries, scores, boxes, labels):
        pboxes = []
        uboxes = []
        for score, box, label in zip(scores, boxes, labels):
            if score < self.SCORE_THRESHOLD:
                continue
            uboxes.append((box, label))
        pboxes = uboxes
        return pboxes, uboxes
    
    def get_preds(self, outputs, inputs, target_sizes):
        logits = torch.max(outputs["logits"][0], dim=-1)
        scores = torch.sigmoid(logits.values).cpu().detach().numpy()
        # Get prediction labels and boundary boxes
        labels = logits.indices.cpu().detach().numpy()
        # target_sizes = torch.tensor([target_sizes])
        self.results = self.processor.post_process_grounded_object_detection(outputs=outputs, 
                                                                             input_ids=inputs.input_ids,
                                                                             target_sizes=target_sizes,
                                                                             box_threshold=self.SCORE_THRESHOLD,
                                                                             text_threshold=self.SCORE_THRESHOLD,
                                                                             )
        boxes  = self.results[0]['boxes'] .detach().numpy()
        pboxes = self.results[0]['boxes'] .detach().numpy()
        scores = self.results[0]['scores'].detach().numpy()
        labels = np.array(self.results[0]['labels'])
        print(labels)
        # sort labels by score, high to low
        sorted_indices = np.argsort(scores)[::-1]

        # store member variables
        # cut off score indices below threshold
        self.sorted_indices = sorted_indices[scores[sorted_indices] > self.SCORE_THRESHOLD]
        self.sorted_scores = scores[self.sorted_indices]
        self.sorted_labels = labels[self.sorted_indices]
        self.sorted_text_labels = labels[self.sorted_indices]
        # self.sorted_text_labels = np.array([self.queries[label] for label in labels[self.sorted_indices]])
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
        self.image = img = np.asarray(input_image)
        img_tensor = torch.tensor(img, dtype=torch.float32)
        # convert input labels from list of labels to single string of period separated elements.
        input_labels = ". ".join(input_labels) + "."
        # inputs = self.processor(input_labels, images=img_tensor, padding=True, return_tensors="pt")
        inputs = self.processor(images=img_tensor, text=input_labels, return_tensors="pt").to(self.device)
        outputs = self.model(**inputs)
        self.dims = img.shape[:2][::-1] # TODO: check if this is correct
        self.W = self.dims[0]
        self.H = self.dims[1]
        # target_sizes = self.dims
        target_sizes = torch.Tensor([[self.H, self.W]])

        self.queries = abbrev_labels
        scores, labels, boxes, pboxes = self.get_preds(outputs, inputs, target_sizes)
        self.plot_predictions(topk=topk, show_plot=plot)
        return self.results, self.sorted_boxes_coords, self.sorted_scores, self.sorted_labels


