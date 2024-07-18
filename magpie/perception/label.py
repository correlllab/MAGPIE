'''
@file label.py
@brief VLM wrapper to retrieve object bounding box and label, given image and input labels
        Additionally, integrate with point cloud data to get 3D bounding boxes
'''
import sys
sys.path.append("../../")
from magpie.perception.object import Object

class Label:
    def __init__(self):
        self.TOP_K = 3
        self.sorted = None
        self.dims = None
        self.H = None
        self.W = None
        self.SCORE_THRESHOLD = 0.01

    def get_boxes(input_image, text_queries, scores, boxes, labels):
        pass

    def get_top_boxes(self, topk=None):
        if topk is None:
            topk = self.TOP_K
        return self.sorted[:topk]

    def get_index(self, index):
        if index < len(self.sorted):
            return self.sorted[index]
        return None

    def label(self, image, labels):
        pass

    def plot_predictions(self, input_image, text_queries, scores, boxes, labels):
        pass

    def get_preds(self, outputs, target_sizes):
        pass
    
    def set_threshold(self, threshold):
        self.SCORE_THRESHOLD = threshold
