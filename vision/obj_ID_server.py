""" 

########## DEV GUIDE ##########

Approach & Guidelines:
* Keep perception and robot motion separate

Input: Python Dictionary
{
    'camera_pose': <list[16]: homog coord>,
}

Outputs: List of {Object Poses & Class Distributions}
[
    ...
    {
        'pose': <list>
        'labels': { ... , <str:label>: <float:confidence> , ... }
    },
    ...
]


########## DEV PLAN ##########
[>] Test perception class as PY file
[ ] Wrap in XML-RPC
    [ ] Test initiating server from Jupyter

"""

########## INIT ####################################################################################

##### Imports #####

import time
import numpy as np
import copy
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from open3d.web_visualizer import draw
import signal

from magpie.perception.label_owlvit import LabelOWLViT

import multiprocessing, logging, os, sys, socket, time, signal, glob
from time import sleep
from multiprocessing import Process, Array, Value, Pool
import numpy as np
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler

##### Camera Start #####

rsc = real.RealSense()
rsc.initConnection()



########## PERCEPTION SERVER, XML-RPC ##############################################################

##### Helper Classes ######################################################

class RequestHandler( SimpleXMLRPCRequestHandler ):
    """ Restrict to a particular path? """
    rpc_paths = ( '/RPC2', )


##### Server ##############################################################

class Perception_OWLViT:
    """ Manage interaction with the OWL-ViT perception stack """

    def __init__( self, num_blocks, query, abbrevq ):
        """ Init perception data structs """
        self.path         = "google/owlvit-base-patch32"
        self.merged_pcd   = o3d.geometry.PointCloud()
        self.query        = query
        self.abbrevq      = abbrevq
        self.blocks       = num_blocks
        self.sleep_rate   = 3
        self.single_query = True
        self.object_pcd   = {}
        self.label_vit    = None
        self.perceiver    = None

    def start_OWLViT( self ):
        """ Load V-LLM """
        # WARNING: Occupies multiple Gb of RAM!
        self.label_vit = LabelOWLViT( pth = self.path )

    def transform_point_cloud(self, cpcd):
        rotation_matrix = np.array([[0, 1, 0, 0],
                                    [-1, 0, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

        tmat_gripper = np.array([[1, 0, 0, -1.15 / 100],
                                 [0, 1, 0, 1.3 / 100],
                                 [0, 0, 1, (309.63 - 195.0) / 1000],
                                 [0, 0, 0, 1]])

        cpcd.transform(tmat_gripper)
        cpcd.transform(rotation_matrix)
        cpcd.transform(self.get_pose())

        return cpcd
    
    def save_point_cloud(self, filename, point_cloud):
         o3d.io.write_point_cloud(filename, point_cloud)

    def save_pose(self, filename, pose):
        np.save(filename, pose)
    
    def load_pose(self, filename):
        return np.load(filename)
    
    def calculate_area(self, box):
        return abs(box[3] - box[1]) * abs(box[2] - box[0])
    
    def find_similar_boxes(self, bounding_boxes, tolerance=1):

        areas = [self.calculate_area(box) for box in bounding_boxes]
        similar_groups = []
        
        for i, area in enumerate(areas):

            similar_boxes = [(i, bounding_boxes[i])]
            for j in range(len(areas)):
                if i != j and abs(areas[j] - area) <= tolerance * area:
                    similar_boxes.append((j, bounding_boxes[j]))
                if len(similar_boxes) >= self.blocks:
                    break
            if len(similar_boxes) >= self.blocks:
                similar_groups.append(similar_boxes)
                if len(similar_groups) == 1:
                    break
        
        if not similar_groups:
            raise ValueError(f"No group with at least {self.blocks} similar areas found.")
                    
        
        return similar_groups
    

    def bound(self, query, abbrevq, use_area_calculation=True):

        p, rgbd_image = rsc.getPCD()
        #crop rgbd image

        
        image = np.array(rgbd_image.color)
        self.label_vit.set_threshold(0.005)
        self.label_vit.TOP_K = 10
        bboxes, uboxes, scores, labels= self.label_vit.label(image, query, abbrevq, topk=True, plot=True)
        #sort scores in descending order 
        scores = sorted(scores, reverse=True)

        objects_of_interest = []
        bounding_boxes = []
        
        slbc = self.label_vit.sorted_labeled_boxes_coords

        for i in range(3):
            # bounding_boxes.append(label_vit.sorted_labeled_boxes_coords[i][0])
            bounding_boxes.append(slbc[i][0])


        if use_area_calculation:
            filtered_boxes = self.find_similar_boxes(bounding_boxes)
            for i, group in enumerate(filtered_boxes):
                for index, box in group:
                    
                    try:
                        cpcd, tmat = self.segment(index, rgbd_image)
                        objects_of_interest.append([cpcd, tmat])
                    except Exception as e:
                        print("Caught TimeoutError:", e)

        return objects_of_interest
   
    
    def segment(self, index, rgbd_image):

        rgbd_image, cpcd, tmat, pcaFrame = pcd.get_segment(
                    self.label_vit.sorted_labeled_boxes_coords, 
                    index, 
                    rgbd_image, 
                    rsc, 
                    type="box", 
                    method="iterative", 
                    display=False,
                    viz_scale=1000)
        
        return cpcd, tmat
    

    def calculate_probability_dist( self, data, scores ):
        probability = {}
        locations = self.find_all_occurrences(data)
        
        # Ensure temp is a list of the same length as self.blocks
        temp = [0] * self.blocks
        
        for i in range(self.blocks):
            sum_scores = 0
            for j in range(len(locations[i])):
                sum_scores += scores[locations[i][j]]
            
            if sum_scores == 0:
                temp[i] = 0
            else:
                temp[i] = scores[locations[i][0]] / sum_scores
        
        # Calculate softmax probabilities
        exp_temp = np.exp(temp)
        softmax = exp_temp / np.sum(exp_temp)
        
        for i in range(self.blocks):  
            probability[self.abbrevq[i]] = softmax[i]
        return probability


    
    def find_all_occurrences(self, data):
        # Initialize a dictionary to store lists of occurrences
        all_occurrences = {obj: [] for obj in range(self.blocks)}
        
        # Iterate through the list and find all occurrences
        print('Initial all_occurrences:', all_occurrences)

        for index, (_, value) in enumerate(data):
            all_occurrences[value].append(index)
        
        print('Final all_occurrences:', all_occurrences)

        return all_occurrences
    

    def get_objects_in_scene( self, n ):
        try:
            # self.start_interface()
            #self.return_to_home()

            for i in range(n):
                if self.single_query:
                    boxes = self.bound(self.query, self.abbrevq)
                    for cpcd, tmat in boxes:
                        cpcd = self.transform_point_cloud(cpcd)


                else:
                    for j in range(len(self.query)):
                        boxes = self.bound(self.query[j], self.abbrevq[j])
                        for cpcd, tmat in boxes:
                            cpcd = self.transform_point_cloud(cpcd)
                            #self.temp_pcd[self.abbrevq[j]][f'Pose {i+1}'] = [cpcd, tmat, score]
                            #temp = self.temp_pcd
                            #print(temp)
        
            
            # self.stop_interface()
        except Exception as e:
            # self.stop_interface()
            raise e


##### XML-RPC Init ########################################################
_QUERIES    = ["a photo of a purple block", "a photo of a blue block", "a photo of a red block"]
_ABBREV_Q   = ["purple", "blue", "red"]
_NUM_BLOCKS = 3

def create_obj_id_xmlrpc_server( configTuple ):
    """ Create an XML-RPC server that is either local or remote """
    # 0. Unpack copnfig
    print( "\n Config Tuple:" , configTuple , '\n' )
    ipad = configTuple['ip']
    port = configTuple['port']
    # 1. Create the XML-RPC object
    server = SimpleXMLRPCServer( ( ipad, port )                    ,
                                 requestHandler = RequestHandler ,
                                 logRequests    = False          )
    # 2. Register the `FT_RegObj` and server query functions
    instance = Perception_OWLViT( _NUM_BLOCKS, _QUERIES, _ABBREV_Q )
    
    server.register_function( instance.get_objects_in_scene )        
    server.register_introspection_functions()

    # 3. Run the server's main loop (This will be done in its own process)
    print( "XML-RPC serving object ID service from", ( ipad, port ) )
    
    server.serve_forever()