import time
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging

from magpie import grasp as gt
from magpie.motor_code import Motors
from magpie import ur5 as ur5
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from magpie.perception.label_owlvit import LabelOWLViT

# Configure logging
logging.basicConfig( level = logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Perception:
    def __init__( self, num_blocks, query, abbrevq, visualize_boxes, visualize_pcd ):
        """
        Initializes the Perception class.

        Args:
            num_blocks (int): Number of blocks.
            query (list): List of queries.
            abbrevq (list): List of abbreviations for the queries.
            visualize_boxes (bool): Flag to visualize bounding boxes.
            visualize_pcd (bool): Flag to visualize point clouds.
        """
        self.ur = ur5.UR5_Interface()
        self.query = query
        self.abbrevq = abbrevq
        self.blocks = num_blocks
        self.visualize_boxes = visualize_boxes
        self.view_pcd = visualize_pcd
        self.sleep_rate = 3
        self.objects = {}

        try:
            self.rsc = real.RealSense()
            self.rsc.initConnection()
        except Exception as e:
            logging.error(f"Error initializing RealSense: {e}")
            raise e
        
        self.label_vit = LabelOWLViT(pth="google/owlvit-base-patch32")

    def start_interface(self):
        """Starts the UR5 interface."""
        try:
            self.ur.start()
        except Exception as e:
            logging.error(f"Error starting UR5 interface: {e}")
            raise e

    def stop_interface(self):
        """Stops the UR5 interface."""
        self.ur.stop()

    def move_to_pose(self, pose):
        """Moves the UR5 to the given pose."""
        self.ur.moveL(pose)
        time.sleep(self.sleep_rate)

    def get_pose(self):
        """Gets the current TCP pose of the UR5."""
        return self.ur.get_tcp_pose()

    def transform_point_cloud(self, cpcd):
        """Transforms the given point cloud to align with the gripper pose."""
        rotation_matrix = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        tmat_gripper = np.array([
            [1, 0, 0, -1.15 / 100],
            [0, 1, 0, 1.3 / 100],
            [0, 0, 1, (309.63 - 195.0) / 1000],
            [0, 0, 0, 1]
        ])

        cpcd.transform(tmat_gripper)
        cpcd.transform(rotation_matrix)
        cpcd.transform(self.get_pose())

        return cpcd

    def save_point_cloud(self, filename, point_cloud):
        """Saves the point cloud to a file."""
        o3d.io.write_point_cloud(filename, point_cloud)

    def save_pose(self, filename, pose):
        """Saves the pose to a file."""
        np.save(filename, pose)

    def load_pose(self, filename):
        """Loads the pose from a file."""
        return np.load(filename)

    def return_to_home(self):
        """Returns the UR5 to the home position."""
        home = self.load_pose("home_pose.npy")
        self.move_to_pose(home)

    def visualize_point_cloud(self, point_cloud):
        """Visualizes the given point cloud."""
        o3d.visualization.draw_geometries([point_cloud])

    def get_pcd_pose(self, point_cloud):
        """Gets the pose of the point cloud."""
        center = point_cloud.get_center()
        pose_vector = [center[0], center[1], center[2], 3.14, 0, 0]
        return pose_vector

    def calculate_area(self, box):
        """Calculates the area of the bounding box."""
        return abs(box[3] - box[1]) * abs(box[2] - box[0])

    def filter_by_area(self, tolerance, box, total_area):
        """Filters the bounding box by area."""
        area = self.calculate_area(box)
        return abs(area / total_area) <= tolerance

    def bound(self, query, abbrevq):
        """Bounds the given query with the OWLViT model."""
        _, rgbd_image = self.rsc.getPCD()
        image = np.array(rgbd_image.color)

        self.label_vit.set_threshold(0.005)
        _, _, scores, labels = self.label_vit.label(image, query, abbrevq, topk=True, plot=False)

        scores = sorted(scores, reverse=True)
        filtered_boxes = []
        filtered_scores = []
        filtered_labels = []
        filter_coords = []

        for i in range(min(20, len(self.label_vit.sorted_labeled_boxes_coords))):
            if self.filter_by_area(0.05, self.label_vit.sorted_labeled_boxes_coords[i][0], image.shape[0] * image.shape[1]):
                filtered_boxes.append(self.label_vit.sorted_boxes[i])
                filtered_scores.append(scores[i])
                filtered_labels.append(labels[i])
                filter_coords.append(self.label_vit.sorted_labeled_boxes_coords[i])

        return rgbd_image, image, abbrevq, filtered_boxes, filtered_scores, filtered_labels, filter_coords

    def calculate_probability_dist(self, cluster):
        """Calculates the probability distribution of the cluster."""
        probabilities = {color: 0 for color in self.abbrevq}
        total = len(cluster)
        color_counts = {color: 0 for color in self.abbrevq}

        for _, color, _, _ in cluster:
            color_counts[color] += 1

        for key in color_counts.keys():
            probabilities[key] = color_counts[key] / total

        return probabilities

    def format_coordinates(self, box):
        """Formats the coordinates of the bounding box."""
        cx, cy, w, h = box
        return [cx - w / 2, cy + h / 2, cx + w / 2, cx - h / 2]

    def find_euclidean_distance(self, point1, point2):
        """Finds the Euclidean distance between two points."""
        return np.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

    def plot_bounding_boxes(self, input_image, scores, boxes, labels, topk=False, show_plot=False):
        """Plots the bounding boxes on the input image."""
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()
        idx = 0

        for i in range(self.blocks):
            for score, box, label in zip(scores[i], boxes[i], labels[i]):
                if score < self.label_vit.SCORE_THRESHOLD:
                    continue
                cx, cy, w, h = box
                ax.plot([cx - w / 2, cx + w / 2, cx + w / 2, cx - w / 2, cx - w / 2],
                        [cy - h / 2, cy - h / 2, cy + h / 2, cy + h / 2, cy - h / 2], "r")
                ax.text(
                    cx - w / 2,
                    cy + h / 2 + 0.015,
                    f"({idx}): {score:1.2f}",
                    ha="left",
                    va="top",
                    color=self.abbrevq[i],
                    bbox={
                        "facecolor": "white",
                        "edgecolor": "red",
                        "boxstyle": "square,pad=.3"
                    })
                idx += 1
            
            fig.canvas.draw()
            if not show_plot:
                plt.close(fig)

    def assign_label(self, k, boundaries):
        """Assigns a label based on the index."""
        output_list = [sum(boundaries[:i + 1]) - 1 for i in range(len(boundaries))]
        for i in range(len(output_list)):
            if k <= output_list[i]:
                return i

    def find_clusters(self, boxes, scores_list):
        """Finds clusters in the bounding boxes."""
        bounding_boxes = []
        boundaries = []
        scores = []

        for i in range(len(boxes)):
            boundaries.append(len(boxes[i]))
            bounding_boxes.extend(boxes[i])
            scores.extend(scores_list[i])

        minw = float('inf')
        for box in bounding_boxes:
            if box[2] < minw:
                minw = box[2]

        clustered_objects = []
        while True:
            clusters = []
            temp = None

            for i in range(len(bounding_boxes)):
                if bounding_boxes[i] is not None:
                    if not clusters:
                        temp = bounding_boxes[i]
                        k = self.assign_label(i, boundaries)
                        clusters.append([bounding_boxes[i], self.abbrevq[k], scores[i], i])
                        bounding_boxes[i] = None
                    else:
                        if self.find_euclidean_distance([temp[0], temp[1]], [bounding_boxes[i][0], bounding_boxes[i][1]]) < minw:
                            k = self.assign_label(i, boundaries)
                            clusters.append([bounding_boxes[i], self.abbrevq[k], scores[i], i])
                            bounding_boxes[i] = None

            clustered_objects.append(clusters)
            if all(x is None for x in bounding_boxes) or len(clusters) == 1:
                break

        return clustered_objects

    def build_model(self):
        """Builds the perception model."""
        try:
            self.start_interface()
            images, abbrevqs, filtered_boxes, filtered_scores, filtered_labels, filtered_coords = [], [], [], [], [], []

            for j in range(len(self.query)):
                rgbd, image, abbrevq, boxes, scores, labels, coords = self.bound(self.query[j], self.abbrevq[j])
                abbrevqs.append(abbrevq)
                filtered_boxes.append(boxes)
                filtered_scores.append(scores)
                filtered_labels.append(labels)
                filtered_coords.append(coords)

            clusters = self.find_clusters(filtered_boxes, filtered_scores)
            self.objects = {f'Object {objectnum + 1}': {} for objectnum in range(len(clusters))}
            index_to_segment = [max(cluster, key=lambda x: x[2])[3] for cluster in clusters]
            formatted_boxes = [box for box_list in filtered_coords for box in box_list]

            for num, index in enumerate(index_to_segment):
                _, cpcd, _, _ = pcd.get_segment(
                    formatted_boxes,
                    index,
                    rgbd,
                    self.rsc,
                    type="box",
                    method="iterative",
                    display=False,
                    viz_scale=1000
                )

                cpcd = self.transform_point_cloud(cpcd)
                if self.view_pcd:
                    self.visualize_point_cloud(cpcd)

                self.objects[f'Object {num + 1}']['Probability'] = self.calculate_probability_dist(clusters[num])
                self.objects[f'Object {num + 1}']['Pose'] = self.get_pcd_pose(cpcd)

            if self.visualize_boxes:
                self.plot_bounding_boxes(image, filtered_scores, filtered_boxes, filtered_labels, topk=False, show_plot=True)

            self.stop_interface()
            return self.objects

        except Exception as e:
            self.stop_interface()
            logging.error(f"Error building model: {e}")
            raise e

# Initialize Perception
queries = ["a photo of a purple block", "a photo of a blue block", "a photo of a red block"]
abbrevq = ["purple", "blue", "red"]
num_blocks = 3
view_combined_boxes_plot = True
visualize_pcd = False

tower = Perception(num_blocks, queries, abbrevq, view_combined_boxes_plot, visualize_pcd)
data = tower.build_model()
print(data)