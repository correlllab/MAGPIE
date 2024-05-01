from py2pddl import Domain, create_type, predicate, action, init, goal
import blocksDomain as bd

class blocksProblem(bd.blocksDomain):
    def __init__(self):
        super().__init__()
        self.objects = bd.blocksDomain.Object.create_objs(
            ["redBlock", "yellowBlock", "blueBlock", "loc-a", "loc-b", "loc-c"], prefix="")

    @init
    def init(self, initDict) -> list:
        # initDicts keys are <predicateName>
        # initDict values are objects the predicate holds for as string
        initState = []
        for predicateName in initDict:
            # Objects the predicate holds for (can be a single string or a list)
            for objects in initDict[predicateName]:
                if predicateName == "on":
                    newPredicate = self.on(self.objects[objects[0]], self.objects[objects[1]])

                elif predicateName == "fixed":
                    newPredicate = self.fixed(self.objects[objects])

                elif predicateName == "clear":
                    newPredicate = self.clear(self.objects[objects])

                elif predicateName == "Block":
                    newPredicate = self.Block(self.objects[objects])
                initState.append(newPredicate)
        return initState

    @goal
    def goal(self, goalDict) -> list:
        # initDicts keys are <predicateName>
        # initDict values are objects the predicate holds for as string
        # initDict values are objects the predicate holds for as string
        goalState = []
        for predicateName in goalDict:
            # Objects the predicate holds for (can be a single string or a list)
            for objects in goalDict[predicateName]:
                if predicateName == "on":
                    newPredicate = self.on(self.objects[objects[0]], self.objects[objects[1]])

                elif predicateName == "fixed":
                    newPredicate = self.fixed(self.objects[objects])

                elif predicateName == "clear":
                    newPredicate = self.clear(self.objects[objects])

                elif predicateName == "Block":
                    newPredicate = self.Block(self.objects[objects])
                goalState.append(newPredicate)
        return goalState
        
        '''
        goalTruths = [
            self.on(self.objects["blueBlock"],self.objects["loc-b"]),
            self.on(self.objects["yellowBlock"],self.objects["blueBlock"]),
            self.on(self.objects["redBlock"],self.objects["loc-c"])
        ]
        '''
        '''
        goalTruths = [
            self.on(self.objects["blueBlock"],self.objects["redBlock"]),
            self.on(self.objects["redBlock"],self.objects["loc-a"]),
            self.on(self.objects["yellowBlock"],self.objects["redBlock"])
        ]
        '''
        '''
        goalTruths = [
            self.on(self.objects["blueBlock"],self.objects["loc-b"])
        ]
        '''
        '''    
        goalTruths = [
            self.on(self.objects["blueBlock"],self.objects["redBlock"])
        ]

        '''
        '''
        goalTruths = [
            self.on(self.objects["redBlock"],self.objects["loc-c"]),
            self.on(self.objects["yellowBlock"],self.objects["redBlock"]),
            self.on(self.objects["blueBlock"],self.objects["yellowBlock"])
        ]
        '''
        return goalTruths