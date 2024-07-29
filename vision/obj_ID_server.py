########## INIT ####################################################################################

##### Imports #####
### Standard ###
import time, logging, gc, os, sys, traceback, multiprocessing
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
# print( os.path.normpath( os.path.join( os.path.dirname( os.path.abspath (__file__ ) ), '..', 'magpie')), flush=True, file=sys.stderr )
sys.path.append( os.path.normpath( os.path.join( os.path.dirname( os.path.abspath (__file__ ) ), '..' )))
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from magpie.perception.label_owlvit import LabelOWLViT
from interprocess import set_non_blocking, non_block_read, PBJSON_IO

os.environ["TOKENIZERS_PARALLELISM"] = "True"

########## PERCEPTION SETTINGS #####################################################################

_QUERIES = [ 
    # "a photo of a violet block", 
    "a photo of a blue block" , 
    # "a photo of a red block"     ,
    "a photo of a yellow block", 
    "a photo of a green block", 
    # "a photo of a orange block",
]
_ABBREV_Q = [
    # "vio", 
    "blu", 
    # "red", 
    "ylw", 
    "grn", 
    # "orn"
]
assert len( _QUERIES ) == len( _ABBREV_Q ), f"ERROR: MISMATCH in number of queries and abbreviated queries!\n{_QUERIES}\n{_ABBREV_Q}\n"
_NUM_BLOCKS  = len( _QUERIES )
_PLOT_BOX    = False
_VIZ_PCD     = False
_ID_PERIOD_S = 2.0
_VERBOSE     = 0




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
    effPose         = np.eye(4).reshape( (16,) ).tolist()
    rotation_matrix = np.array([
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]
    ])
    tmat_gripper = np.array([
        [1, 0, 0, -1.15 / 100        ],
        [0, 1, 0,  1.3 / 100         ],
        [0, 0, 1, (309.63-195.0)/1000],
        [0, 0, 0, 1                  ]
    ])

    captured_data = [] ################################################## NEW LIST TO SAVE DATA ACROSS RUNS


    @classmethod
    def set_update_period( cls, per_ ):
        cls.per = per_


    @classmethod
    def start_vision( cls ):
        try:
            cls.rsc = real.RealSense()
            cls.rsc.initConnection()
            if _VERBOSE:
                print( f"RealSense camera CONNECTED", flush=True, file=sys.stderr )
        except Exception as e:
            if _VERBOSE:
                print( f"\nERROR initializing RealSense: {e}\n", flush=True, file=sys.stderr )
            raise e
        
        try:
            cls.label_vit = LabelOWLViT( pth = "google/owlvit-base-patch32" )
            if _VERBOSE:
                print( f"V-LLM STARTED", flush=True, file=sys.stderr )
        except Exception as e:
            if _VERBOSE:
                print( f"\nERROR initializing OWL-ViT: {e}\n", flush=True, file=sys.stderr )
            raise e
        
    @classmethod
    def shutdown( cls ):
        try:
            cls.rsc.disconnect()
            if _VERBOSE:
                print( f"RealSense camera DISCONNECTED", flush=True, file=sys.stderr )
        except Exception as e:
            if _VERBOSE:
                print( f"\nERROR disconnecting RealSense: {e}\n", flush=True, file=sys.stderr )
            raise e
        
        try:
            del cls.label_vit 
            cls.label_vit = None
            gc.collect()
            if _VERBOSE:
                print( f"V-LLM SHUTDOWN", flush=True, file=sys.stderr )
        except Exception as e:
            if _VERBOSE:
                print( f"\nERROR cleaning OWL-ViT: {e}\n", flush=True, file=sys.stderr )
            raise e


    @classmethod
    def get_corrected_gripper_pose( cls ):
        if 0:
            matx = np.eye(4)
            matx = matx.dot( cls.tmat_gripper )
            matx = matx.dot( cls.rotation_matrix )
            matx = matx.dot( np.asarray( cls.effPose ).reshape( (4,4,) ) )
        else:
            matx = np.asarray( cls.effPose ).reshape( (4,4,) )
            matx = matx.dot( cls.rotation_matrix )
            matx = matx.dot( cls.tmat_gripper )
        return matx

    @classmethod
    def transform_point_cloud( cls, cpcd ):
        """Transforms the given point cloud to align with the gripper pose."""
        # cpcd.transform( cls.tmat_gripper )
        # cpcd.transform( cls.rotation_matrix )
        # #convert to 4*4
        # cpcd.transform( np.asarray( cls.effPose[:] ).reshape( (4,4,) ) )
        cpcd.transform( cls.get_corrected_gripper_pose() )
        # return cpcd
    
    @classmethod
    def save_point_cloud(cls, filename, point_cloud):
        """Saves the point cloud to a file."""
        o3d.io.write_point_cloud(filename, point_cloud)

    @classmethod
    def get_pcd_pose( cls, point_cloud ):
        """Gets the pose of the point cloud."""
        center = point_cloud.get_center()

        # pose_vector = [center[0], center[1], center[2], 3.14, 0, 0]
        # HACK: HARDCODED ORIENTATION
        # FIXME: GET THE "ACTUAL" ORIENTATION VIA PCA
        pose_vector = np.eye(4)
        for i in range(3):
            pose_vector[i,3] = center[i]
        return pose_vector.reshape( (16,) ).tolist()

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
    def bound( cls, query, abbrevq ):
        """Bounds the given query with the OWLViT model."""
        _, rgbd_image = cls.rsc.getPCD()
        image = np.array( rgbd_image.color )

        cls.label_vit.set_threshold(0.005)
        _, _, scores, labels = cls.label_vit.label(image, query, abbrevq, topk=True, plot=False)

        scores = sorted(scores, reverse=True)
        filtered_boxes = []
        filtered_scores = []
        filtered_labels = []
        filter_coords = []

        # cls.label_vit.

        # print( type( cls.label_vit.sorted_labeled_boxes_coords ) )
        # exit()

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

            if 0:
                # HACK: LESS FUCKING PURPLE AND BLUE
                if color == 'vio':
                    color_counts[color] += 0.3
                elif color == 'blu':
                    color_counts[color] += 0.4
                else:
                    color_counts[color] += 1
            else:
                # Alt: Just get closer and use fewer blocks
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
    
    ############################ NEW FUNCTIONS BELOW FOR MULTI POSE DATA CAPTURE #######################################################
    @classmethod 
    def capture_image(cls):
        try:
            # rgbd_image, _ = cls.rsc.getPCD()
            pcd, rgbd_image  = cls.rsc.getPCD()
            # cls.captured_data.append(rgbd_image)
            cls.captured_data.append( [pcd, rgbd_image,] )
            return True
        except Exception as e:
            logging.error(f"Error capturing image: {e}")
            return False
        
    @classmethod  
    def merge_and_build_model(cls):
        try:
            if not len( cls.captured_data ):
                raise ValueError("No captured data to merge.")

            merged_pcd = None

            for (pcd_i, rgbd_i) in cls.captured_data:
                if merged_pcd is None:
                    # merged_pcd = data
                    merged_pcd = pcd_i
                else:
                    merged_pcd += pcd_i

            cls.captured_data = []  # Clear captured data after merging
            if _VERBOSE: 
                print( f"\nAbout to merge {type(merged_pcd)} {merged_pcd} ... \n" )
            return cls.build_model( merged_pcd )
        except Exception as e:
            logging.error(f"\nError merging data and building model: {e}\n")
            traceback.print_exc()
            return {}
        
    @classmethod
    def capture_and_return_observations(cls):
        try:
            rgbd_image, _ = cls.rsc.getPCD()
            return cls.build_model(rgbd_image)
        except Exception as e:
            logging.error(f"Error capturing image and returning observations: {e}")
            return {}

    @classmethod
    def build_model(cls, rgbd_image=None): ###ADDED INPUT 
        """Builds the perception model."""

        cls.label_vit.reset_prediction_state()

        print( f"\nInside `build_model`\n", flush=True, file=sys.stderr )

        try:
            images, abbrevqs, filtered_boxes, filtered_scores, filtered_labels, filtered_coords = [], [], [], [], [], []

            for j in range(len(_QUERIES)):
                rgbd, image, abbrevq, boxes, scores, labels, coords = cls.bound(_QUERIES[j], _ABBREV_Q[j])

                if rgbd_image:    ####### CONDITION TO REPLACE RGBD IF INPUT IS NOT NONE
                    rgbd = rgbd_image
                
                abbrevqs.append(abbrevq)
                filtered_boxes.append(boxes)
                filtered_scores.append(scores)
                filtered_labels.append(labels)
                filtered_coords.append(coords)

            if _VERBOSE:
                print( f"\nAbout to build clusters ...\n", flush=True, file=sys.stderr )

            clusters = cls.find_clusters(filtered_boxes, filtered_scores)
            clusters = [item for item in clusters if len( item )] # 2024-07-28: Some clusters are EMPTY
            objIDstrTemp = {f'Object {objectnum + 1}': {} for objectnum in range(len(clusters))}

            index_to_segment = [max(cluster, key=lambda x: x[2])[3] for cluster in clusters]
            formatted_boxes = [box for box_list in filtered_coords for box in box_list]

            if _VERBOSE:
                print( f"\nIs there a depth image?: {rgbd}\n", flush=True, file=sys.stderr )
                print( f"\nAre there boxes?: {formatted_boxes}\n", flush=True, file=sys.stderr )

            for num, index in enumerate(index_to_segment):

                if _VERBOSE:
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
                    if _VERBOSE:
                        print( f"\nPCD {cpcd} ...\n", flush=True, file=sys.stderr )
                except Exception as e:
                    if _VERBOSE:
                        print(f"Segmentation error: {e}", flush=True, file=sys.stderr)
                    raise e
                
                # try:
                #     if _VERBOSE:
                #         print( f"\nAbout to transform PCD ...\n", flush=True, file=sys.stderr )

                #     cls.transform_point_cloud( cpcd )
                #     if cls.view_pcd:
                #         cls.visualize_point_cloud( cpcd )
                # except Exception as e:
                #     if _VERBOSE:
                #         print(f"Transformation error: {e}", flush=True, file=sys.stderr)
                #     raise e

                objIDstrTemp[f'Object {num + 1}']['Probability'] = cls.calculate_probability_dist(clusters[num])
                objIDstrTemp[f'Object {num + 1}']['Pose']        = cls.get_pcd_pose( cpcd )
                objIDstrTemp[f'Object {num + 1}']['Count']       = len( clusters[num] )
                objIDstrTemp[f'Object {num + 1}']['Time']        = now()

            if cls.visualize_boxes:
                cls.plot_bounding_boxes(image, filtered_scores, filtered_boxes, filtered_labels, topk=False, show_plot=True)

                
            return objIDstrTemp

        except Exception as e:
            print(f"Error building model: {e}", flush=True, file=sys.stderr)
            traceback.print_exc()
            raise e
        
        except KeyboardInterrupt as e:
            print( f"\n`build_model` was stopped by user: {e}\n", flush=True, file=sys.stderr )
            raise e



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
                    Perception_OWLViT.effPose = np.array( msg['data'] ).reshape( (4,4,) )
                else:
                    print( f"\nPerception received a POORLY FORMED command!:\n{msg}\n", flush=True, file=sys.stderr )
            else:
                print( f"\nPerception received a POORLY FORMED command!:\n{msg}\n", flush=True, file=sys.stderr )

    

        ##### Vision #############################

        # D. Fetch the pose+image  &&  Identify objects in the scene  &&  Create output message  &&  Send
        if vizFlag:
            if _VERBOSE:
                print( f"\nVision was ENABLED\n", flush=True, file=sys.stderr )
            # Perception_OWLViT.effPose = effPose.copy()
            data = Perception_OWLViT.build_model()
            if _VERBOSE:
                print( f"\nAbout to pack data: {data}\n", flush=True, file=sys.stderr )
            sys.stdout.buffer.write( pbj.pack( data ) )
            sys.stdout.buffer.flush()
            data = {}
        

        ##### Rate Limit #########################
        elapsed = now() - lstLoop
        if (elapsed < Perception_OWLViT.per):
            sleep( Perception_OWLViT.per - elapsed )
        lstLoop = now()


    print( "\nPerception Process will DIE\n", flush=True, file=sys.stderr )
    os.system( 'kill %d' % os.getpid() ) 

