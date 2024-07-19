########## INIT ####################################################################################

##### Imports #####
### Standard ###
import time, logging, ctypes, os, sys, traceback, multiprocessing
now = time.time
from time import sleep
from ast import literal_eval
from pprint import pprint
from multiprocessing import Process, Array, Value, RawArray, Manager

### Special ###
import numpy as np
from PIL import Image
import open3d as o3d
from open3d.web_visualizer import draw
import matplotlib.pyplot as plt

### Local ###
sys.path.append( "../" )
from magpie import grasp as gt
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from magpie.perception.label_owlvit import LabelOWLViT
from interprocess import set_non_blocking, non_block_read, PBJSON_IO



########## PERCEPTION SETTINGS #####################################################################

_QUERIES     = ["a photo of a purple block", "a photo of a blue block", "a photo of a red block"]
_ABBREV_Q    = ["purple", "blue", "red"]
_NUM_BLOCKS  = 3
_PLOT_BOX    = False
_VIZ_PCD     = False
_ID_PERIOD_S = 2.0
_STR_MAX     = 2048



########## PERCEPTION SERVER, MULTIPROCESSING ######################################################


##### Server ##############################################################
# Configure logging
logging.basicConfig( level = logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')



class Perception_OWLViT:
    """ Perception service based on OWL-ViT """

    ### Class Vars ###

    query           = _QUERIES
    abbrevq         = _ABBREV_Q
    blocks          = _NUM_BLOCKS
    visualize_boxes = _PLOT_BOX
    view_pcd        = _VIZ_PCD
    rsc             = None
    label_vit       = None 
    lst             = 0.0 #------------------------------------------ Last time loop ended,  INSIDE  Process
    per             = _ID_PERIOD_S #--------------------------------- Period [s], __________ INSIDE  Process
    effPose         = Array( 'd', np.eye(4).reshape( (16,) ).tolist() )

    # objIDstr        = Array( 'c', bytes( "Hello, World!", 'utf-8' ) )
    # objIDstr        = Array( 'c_wchar_p', "Hello, World!" )
    objIDstr        = Array( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )
    # objIDstr        = RawArray( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )
    # objIDstr        = Array( ctypes.c_byte, [0 for _ in range( _STR_MAX )] )
    # objIDstr        = Manager().list( [b'\x00' for _ in range( _STR_MAX )] )

    # objIDstr        = Value( ctypes.c_char_p, b"Hello, World!" )
    # objIDstr        = Value( ctypes.c_wchar_p, "Hello, World!" )

    run             = Value( 'B', 1 )
    viz             = Value( 'B', 1 )

    @classmethod
    def start_vision( cls ):

        # cls.effPose  = Array( 'd', np.eye(4).reshape( (16,) ).tolist() )
        # cls.objIDstr = Array( ctypes.c_char, [b'\x00' for _ in range( _STR_MAX )] )

        try:
            cls.rsc = real.RealSense()
            cls.rsc.initConnection()
            # logging.info( f"RealSense camera CONNECTED" )
            print( f"RealSense camera CONNECTED", flush=True, file=sys.stderr )
        except Exception as e:
            # logging.error( f"Error initializing RealSense: {e}" )
            print( f"\nERROR initializing RealSense: {e}\n", flush=True, file=sys.stderr )
            raise e
        
        try:
            cls.label_vit = LabelOWLViT( pth = "google/owlvit-base-patch32" )
            # logging.info( f"V-LLM STARTED" )
            print( f"V-LLM STARTED", flush=True, file=sys.stderr )
        except Exception as e:
            # logging.error( f"Error initializing OWL-ViT: {e}" )
            print( f"\nERROR initializing OWL-ViT: {e}\n", flush=True, file=sys.stderr )
            raise e

    @classmethod
    def transform_point_cloud(cls, cpcd):
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
        #convert to 4*4
        cpcd.transform( np.asarray( cls.effPose[:] ).reshape( (4,4,) ) )

        return cpcd
    
    @classmethod
    def save_point_cloud(cls, filename, point_cloud):
        """Saves the point cloud to a file."""
        o3d.io.write_point_cloud(filename, point_cloud)

    @classmethod
    def get_pcd_pose(cls, point_cloud):
        """Gets the pose of the point cloud."""
        center = point_cloud.get_center()
        pose_vector = [center[0], center[1], center[2], 3.14, 0, 0]
        return pose_vector

    @classmethod
    def calculate_area(cls, box):
        """Calculates the area of the bounding box."""
        return abs(box[3] - box[1]) * abs(box[2] - box[0])

    @classmethod
    def filter_by_area(cls, tolerance, box, total_area):
        """Filters the bounding box by area."""
        area = cls.calculate_area(box)
        return abs(area / total_area) <= tolerance

    @classmethod
    def bound(cls, query, abbrevq):
        """Bounds the given query with the OWLViT model."""
        _, rgbd_image = cls.rsc.getPCD()
        image = np.array(rgbd_image.color)

        cls.label_vit.set_threshold(0.005)
        _, _, scores, labels = cls.label_vit.label(image, query, abbrevq, topk=True, plot=False)

        scores = sorted(scores, reverse=True)
        filtered_boxes = []
        filtered_scores = []
        filtered_labels = []
        filter_coords = []

        for i in range(min(20, len(cls.label_vit.sorted_labeled_boxes_coords))):
            if cls.filter_by_area(0.05, cls.label_vit.sorted_labeled_boxes_coords[i][0], image.shape[0] * image.shape[1]):
                filtered_boxes.append(cls.label_vit.sorted_boxes[i])
                filtered_scores.append(scores[i])
                filtered_labels.append(labels[i])
                filter_coords.append(cls.label_vit.sorted_labeled_boxes_coords[i])

        return rgbd_image, image, abbrevq, filtered_boxes, filtered_scores, filtered_labels, filter_coords

    @classmethod
    def calculate_probability_dist(cls, cluster):
        """Calculates the probability distribution of the cluster."""
        probabilities = {color: 0 for color in _ABBREV_Q}
        total = len(cluster)
        color_counts = {color: 0 for color in _ABBREV_Q}

        for _, color, _, _ in cluster:
            color_counts[color] += 1

        for key in color_counts.keys():
            probabilities[key] = color_counts[key] / total

        return probabilities

    @classmethod
    def format_coordinates(cls, box):
        """Formats the coordinates of the bounding box."""
        cx, cy, w, h = box
        return [cx - w / 2, cy + h / 2, cx + w / 2, cx - h / 2]

    @classmethod
    def find_euclidean_distance(cls, point1, point2):
        """Finds the Euclidean distance between two points."""
        return np.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

    @classmethod
    def plot_bounding_boxes(cls, input_image, scores, boxes, labels, topk=False, show_plot=False):
        """Plots the bounding boxes on the input image."""
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.imshow(input_image, extent=(0, 1, 1, 0))
        ax.set_axis_off()
        idx = 0

        for i in range(_NUM_BLOCKS):
            for score, box, label in zip(scores[i], boxes[i], labels[i]):
                if score < cls.label_vit.SCORE_THRESHOLD:
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
                    color=_ABBREV_Q[i],
                    bbox={
                        "facecolor": "white",
                        "edgecolor": "red",
                        "boxstyle": "square,pad=.3"
                    })
                idx += 1
            
            fig.canvas.draw()
            if not show_plot:
                plt.close(fig)

    @classmethod
    def assign_label(cls, k, boundaries):
        """Assigns a label based on the index."""
        output_list = [sum(boundaries[:i + 1]) - 1 for i in range(len(boundaries))]
        for i in range(len(output_list)):
            if k <= output_list[i]:
                return i

    @classmethod
    def find_clusters(cls, boxes, scores_list):
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
                        k = cls.assign_label(i, boundaries)
                        clusters.append([bounding_boxes[i], _ABBREV_Q[k], scores[i], i])
                        bounding_boxes[i] = None
                    else:
                        if cls.find_euclidean_distance([temp[0], temp[1]], [bounding_boxes[i][0], bounding_boxes[i][1]]) < minw:
                            k = cls.assign_label(i, boundaries)
                            clusters.append([bounding_boxes[i], _ABBREV_Q[k], scores[i], i])
                            bounding_boxes[i] = None

            clustered_objects.append(clusters)
            if all(x is None for x in bounding_boxes) or len(clusters) == 1:
                break

        return clustered_objects

    @classmethod
    def build_model(cls):
        """Builds the perception model."""

        print( f"\nInside `build_model`\n", flush=True, file=sys.stderr )

        try:
            images, abbrevqs, filtered_boxes, filtered_scores, filtered_labels, filtered_coords = [], [], [], [], [], []

            for j in range(len(_QUERIES)):
                rgbd, image, abbrevq, boxes, scores, labels, coords = cls.bound(_QUERIES[j], _ABBREV_Q[j])
                
                abbrevqs.append(abbrevq)
                filtered_boxes.append(boxes)
                filtered_scores.append(scores)
                filtered_labels.append(labels)
                filtered_coords.append(coords)

            print( f"\nAbout to build clusters ...\n", flush=True, file=sys.stderr )

            clusters = cls.find_clusters(filtered_boxes, filtered_scores)
            objIDstrTemp = {f'Object {objectnum + 1}': {} for objectnum in range(len(clusters))}

            index_to_segment = [max(cluster, key=lambda x: x[2])[3] for cluster in clusters]
            formatted_boxes = [box for box_list in filtered_coords for box in box_list]

            print( f"\nIs there a depth image?: {rgbd}\n", flush=True, file=sys.stderr )
            print( f"\nAre there boxes?: {formatted_boxes}\n", flush=True, file=sys.stderr )

            for num, index in enumerate(index_to_segment):

                print( f"\nNumber {num}, Index {index}, Camera {cls.rsc} ...\n", flush=True, file=sys.stderr )

                cpcd = None

                try:
                    _, cpcd, _, _ = pcd.get_segment(
                        formatted_boxes,
                        index,
                        rgbd,
                        cls.rsc,
                        type="box",
                        method="iterative",
                        display=False,
                        viz_scale=1000
                    )
                    print( f"\nPCD {cpcd} ...\n", flush=True, file=sys.stderr )
                except Exception as e:
                    # logging.error(f"Error building model: {e}", flush=True, file=sys.stderr)
                    print(f"Segmentation error: {e}", flush=True, file=sys.stderr)
                    raise e
                
                try:
                    print( f"\nAbout to transform PCD ...\n", flush=True, file=sys.stderr )

                    cpcd = cls.transform_point_cloud( cpcd )
                    if cls.view_pcd:
                        cls.visualize_point_cloud( cpcd )
                except Exception as e:
                    # logging.error(f"Error building model: {e}", flush=True, file=sys.stderr)
                    print(f"Transformation error: {e}", flush=True, file=sys.stderr)
                    raise e

                objIDstrTemp[f'Object {num + 1}']['Probability'] = cls.calculate_probability_dist(clusters[num])
                objIDstrTemp[f'Object {num + 1}']['Pose'] = cls.get_pcd_pose(cpcd)

            if cls.visualize_boxes:
                cls.plot_bounding_boxes(image, filtered_scores, filtered_boxes, filtered_labels, topk=False, show_plot=True)

                
            # cls.objIDstr.value = bytes(objIDstrTemp , 'utf-8')
            return objIDstrTemp

        except Exception as e:
            # logging.error(f"Error building model: {e}", flush=True, file=sys.stderr)
            print(f"Error building model: {e}", flush=True, file=sys.stderr)
            raise e
        
        except KeyboardInterrupt as e:
            # logging.error( f"Program was stopped by user: {e}", flush=True, file=sys.stderr )
            print( f"Program was stopped by user: {e}", flush=True, file=sys.stderr )
            traceback.print_exc()



########## MAIN ####################################################################################

if __name__ == "__main__":

    # 0. Set the process context so that Open3D does not randomly hang and/or crash
    # https://github.com/isl-org/Open3D/issues/4007#issuecomment-940996106
    multiprocessing.set_start_method( 'forkserver', force = True )

    # 1. Setup stdout/in JSON message passing
    set_non_blocking( sys.stdin )
    pbj = PBJSON_IO()
    sleep( 0.100 )

    # 2. Start RealSense and OWL-ViT, NOTE: Parent process should wait 20s for massive model to load
    Perception_OWLViT.start_vision()


    ##### 3. Perception Loop ##############################################
    effPose = np.eye(4)
    lstLoop = now()
    vizFlag = True
    runFlag = True

    while runFlag:

        ##### Input ##############################
        # A. Receive all available bytes
        inpt = non_block_read( sys.stdin.buffer )
        pbj.write( inpt )

        # B. Decode any complete messages that have accumulated
        if pbj.unpack():
            inbox = pbj.get_all()
        else:
            inbox = []

        # C. If messages exist, then handle them
        for msg in inbox:
            print( f"Message Received:\n{msg}\n", flush=True, file=sys.stderr )
            if ('cmnd' in msg):
                if (msg['cmnd'] == "SHUTDOWN"):
                    print( "\nDONE\n", flush=True, file=sys.stderr )
                    runFlag = False
                elif (msg['cmnd'] == "SET_VIZ"):
                    vizFlag = bool( msg['data'] )
                elif (msg['cmnd'] == "POSE_IN"):
                    effPose = np.array( msg['data'] ).reshape( (4,4,) )
                else:
                    print( f"\nPerception received a POORLY FORMED command!:\n{msg}\n", flush=True, file=sys.stderr )
            else:
                print( f"\nPerception received a POORLY FORMED command!:\n{msg}\n", flush=True, file=sys.stderr )

    

        ##### Vision #############################

        # D. Fetch the pose+image  &&  Identify objects in the scene  &&  Create output message  &&  Send
        if vizFlag:
            print( f"\nVision was ENABLED\n", flush=True, file=sys.stderr )
            Perception_OWLViT.effPose = effPose.copy()
            data = Perception_OWLViT.build_model()
            print( f"\nAbout to pack data: {data}\n", flush=True, file=sys.stderr )
            sys.stdout.buffer.write( pbj.pack( data ) )
            sys.stdout.buffer.flush()
        

        ##### Rate Limit #########################
        elapsed = now() - lstLoop
        if (elapsed < Perception_OWLViT.per):
            sleep( Perception_OWLViT.per - elapsed )
        lstLoop = now()


    print( "\nPerception Process will DIE\n", flush=True, file=sys.stderr )
    os.system( 'kill %d' % os.getpid() ) 

