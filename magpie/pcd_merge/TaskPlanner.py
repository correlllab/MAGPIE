# Task Planner Code Continues
import blocksProblem as bp
import os
import copy
import numpy as np

class TaskPlanner():
    def __init__(self, blocks):
        self.blocks = blocks
        self.blockConnections = None
        # loc's will have same positions as blocks, but they should really be below them which requires projection
        self.locPositions = {}  # dictionary that maps from location names to gripper frame coordinates
        pass

    def blockOn(self, blockA, blockB):
        # returns true if blockA is on blockB based on rough world coordinates
        # returning false implies that block is on the table
        # CONDITIONALS MAY NEED TO CHANGE IF ACTUAL UR5 COORDINATE SYSTEM DOESN'T MATCH DRAWING (FOR INSTANCE CHECK BOUNDS IN X RATHER THAN Y)
        aX, aY, aZ = blockA.worldFrameCoords
        bX, bY, bZ = blockB.worldFrameCoords
        # CHANGE BLOCK LENGTH FOR REAL WORLD EXPERIMENTS WITH DIFFERENT SIZED BLOCK (0.1)
        blockLength = 0.02  # meters (20 mm)
        # Check if the distance between the centers is within 60% of the blockLength in the x-axis
        if abs(bX - aX) <= (0.5 * blockLength * 1.2):
            # check if z-axis distance between the centers is within 150% of the blockLength with block a being on top of b
            if abs(bZ - aZ) <= (1.5 * blockLength) and aZ > bZ:
                return True
        return False

    def getProblemArguments(self, blocks):
        # returns tuple of (initDict,locPositions)
        # initDict contains values passed into init
        # locPositions is a dict that maps from loc name to gripperFrameCoordinates
        blockPairs = set()
        for blockA in blocks:
            for blockB in blocks:
                if blockA != blockB:
                    blockPairs.add((blockA, blockB))

        blockConnectionsBelow = {}  # graph as dict mapping from block to block below it
        for blockA, blockB in blockPairs:
            if self.blockOn(blockA, blockB):
                blockConnectionsBelow[blockA.name] = blockB.name

        # print(blockConnectionsBelow)
        # This would randomly assign loc to be below bottom level blocks but this will be hard-coded
        # Order from left to right based on world coordinate X (loc-a is at xMax)
        tablePos = ["loc-a", "loc-b", "loc-c"]
        bottomBlocks = []  # list containing tuples of lowest left blocks and their x position
        for block in blocks:
            if block.name not in blockConnectionsBelow:
                # if a block has nothing below it
                bottomBlocks.append((block.worldFrameCoords[0], block))

        # Order bottom level blocks by decreasing x value
        bottomBlocks.sort()
        bottomBlocks.reverse()


        # Method assumes there is enough room between the left and rightmost blocks for loc b (cannot have a stack only in locA,locB with locC on right and empty)
        if len(bottomBlocks) >= 2:
            # If there are 2-3 blocks at the bottom assign loc-a to leftmost, loc-c to rightmost
            leftBlock = bottomBlocks[0][1]
            rightBlock = bottomBlocks[-1][1]
            blockConnectionsBelow[leftBlock.name] = "loc-a"
            blockConnectionsBelow[rightBlock.name] = "loc-c"
            self.locPositions["loc-a"] = leftBlock.gripperFrameCoords
            self.locPositions["loc-c"] = rightBlock.gripperFrameCoords

            if len(bottomBlocks) == 3:
                # If there are 3 blocks at bottom also assign loc-b to middle blocks
                middleBlock = bottomBlocks[1][1]
                blockConnectionsBelow[middleBlock.name] = "loc-b"
                self.locPositions["loc-b"] = middleBlock.gripperFrameCoords
            else:
                # If 2 bottom blocks only then loc-b is between left and right block
                l = np.array([leftBlock.gripperFrameCoords, rightBlock.gripperFrameCoords])
                x, y, z = np.mean(l[:, 0]), np.mean(l[:, 1]), np.mean(l[:, 2])
                self.locPositions["loc-b"] = np.array([x, y, z])

        blockConnectionsBelow["loc-a"] = None
        blockConnectionsBelow["loc-b"] = None
        blockConnectionsBelow["loc-c"] = None

        # print(blockConnectionsBelow)

        # problemClass = blocksProblem()
        # problemClass.objects = blocksDomain.Object.create_objs(["redBlock","yellowBlock","blueBlock","loc-a","loc-b","loc-c"],prefix="")

        initDict = {}  # Initially true predicates
        # Keys in initDict correspond to parameter names in the blockDomain.init() function
        # with the value stored under the key passed into init() with the name of the key
        # assert that some objects are of type block
        # Here, keys are predicateNames and values are lists of of objects they hold for
        # This is used for interfacing with Py2PDDL and initializing the state in the problemClass

        predicateNames = ['Block', 'on', 'fixed', 'clear']
        # Each predicate initially acts on no objects []
        for predicateName in predicateNames:
            initDict[predicateName] = []

        initDict['Block'] = ['redBlock', 'blueBlock', 'yellowBlock']
        print("<<INITIAL WORLD STATE>>")
        for Object in blockConnectionsBelow:
            objectBelow = blockConnectionsBelow[Object]
            if objectBelow != None:
                print(f"on({Object},{objectBelow})")
                initDict["on"].append((Object, objectBelow))

            else:
                # bottom level object has objectBelow=None and is location therefore has 'fixed' property
                print(f"fixed({Object})")
                initDict["fixed"].append(Object)

            # check if block is the top layer block i.e not the child (below) any other blocks
            if not Object in blockConnectionsBelow.values():
                print(f"clear({Object})")
                initDict['clear'].append(Object)

        # print(f"initDict:\n{initDict}")
        return initDict

    def generatePDDLFiles(self, blocks):
        # generates domain.pddl and problem.pddl files based on the problem object and superclass which specifies domain
        initDict = self.getProblemArguments(blocks)
        problem = bp.blocksProblem()
        problem.generate_domain_pddl()
        problem.generate_problem_pddl(
            init={
                'initDict': initDict
            },
            goal={
                'goalDict': self.goalDict
            }
        )

    def generatePDDLPlan(self):
        # precondition: call generatePDDLFiles prior to calling this
        # generates plan based on domain.pddl and problem.pddl using fast downward which is writtent to sas_plan file
        print("<<RUNNING FASTDOWNWARD PLANNER>>")
        os.system("/home/andreamiller/ris/downward/fast-downward.py domain.pddl problem.pddl --search 'astar(lmcut())'")

    def parsePlan(self, blocks):
        # precondition: call generatePDDLPlan prior to calling this
        # parses outputted plan in the sas_plan file into a sequence of coordinates to visit
        # planFile = open("./sas_plan", "r")
        planFile = open("./sas_plan", "r")
        fileText = planFile.read()
        planFile.close()
        steps = []
        # coordTranslator = getTranslator(blocks)

        # dictionary with keys equal to object names and values equal to Object class
        # only blocks for now, would be good to expand to locations as well
        nameMap = {}
        for block in self.blocks:
            nameMap[block.name] = block

        print("<<PLANNER OUTPUT>>")
        for actionLine in fileText.split("\n")[0:-2]:
            actionLine = actionLine.replace("(", "")
            actionLine = actionLine.replace(")", "")
            # only can parse move commands for now as number of params varies for other actions
            # compensating with fast downward converting to lowercase
            # can fix with refactor
            actionLine = actionLine.replace("block", "Block")
            # print(actionLine.split(" "))
            actionName, objectToMove, whereToMoveFrom, whereToMoveTo = actionLine.split(" ")
            # print(f"Action: {actionName}\nobjectToMove: {objectToMove}\nUnder Object: {whereToMoveFrom}\nTo: {whereToMoveTo}")
            print(f"Action: {actionName}\nobjectToMove: {objectToMove}\nTo: {whereToMoveTo}")
            blockToMove = copy.deepcopy(nameMap[objectToMove])  # The block that needs to be moved
            pickupCoords = copy.deepcopy(blockToMove.gripperFrameCoords)
            print(f"Pickup {blockToMove.name} at {pickupCoords}")
            # if whereToMoveTo is a block vs. a location handle it different
            # if a location then don't add the block height
            if whereToMoveTo in ["loc-a", "loc-b", "loc-c"]:
                # This extra distance shouldn't be needed but we'll add it anyways
                releaseCoords = copy.deepcopy(self.locPositions[whereToMoveTo])
                print(f"Release on object {whereToMoveTo} at position {releaseCoords}")
            else:
                goalBlock = copy.deepcopy(nameMap[whereToMoveTo])
                # move to a block then add block length to position loc's are already adjusted)
                goalPostion = copy.deepcopy(nameMap[whereToMoveTo])
                blockLength = 0.02
                releaseCoords = goalBlock.gripperFrameCoords 
                print(f"Release on object {whereToMoveTo} at position {releaseCoords}")
            # After moving a block we need to update its position
            blockToMove.gripperFrameCoords = copy.deepcopy(releaseCoords)

            # Steps contains a sequence of pickup and release coordinates
            steps.append((pickupCoords, releaseCoords))
            '''
            pickupCoords = coordTranslator[objectToMove]
            releaseCoords = coordTranslator[whereToMoveTo]
            # drop just above block
            blockLength = 0.02
            releaseCoords[2] -= blockLength
            print(f"Pick up {objectToMove} at {pickupCoords}, Release at {releaseCoords}\n")
            steps.append((pickupCoords,releaseCoords))
            # update object location reference
            coordTranslator[objectToMove] = releaseCoords
            '''
        return steps

    def generatePlan(self, goalDict):
        # Returns a sequence of positions that blocks should be grasped and then released relative to the current gripper frame
   
        try:
            for block in self.blocks:
                print(f"{block.name}: {block.worldFrameCoords * 1000}")
        except:
            print("failed here")

        print("we got here")
        self.goalDict = goalDict
        self.generatePDDLFiles(self.blocks)
        self.generatePDDLPlan()
        steps = self.parsePlan(self.blocks)
        
        return steps
