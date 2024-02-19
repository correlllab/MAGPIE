'''
@file label.py
@brief VLM wrapper to retrieve object bounding box and label, given image and input labels
        Additionally, integrate with point cloud data to get 3D bounding boxes
'''

class Label:
    def __init__(self):
        pass
    
    def get_boxes(input_image, text_queries, scores, boxes, labels):
        pass

    def label(self, image, labels):
        pass

    def plot_predictions(self, input_image, text_queries, scores, boxes, labels):
        pass

    def get_preds(self, outputs, target_sizes):
        pass
    
    def set_threshold(self, threshold):
        pass