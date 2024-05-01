from py2pddl import Domain, create_type, predicate, action, init, goal


class blocksDomain(Domain):
    Object = create_type("Object")

    @predicate(Object)
    def Block(self, objectA):
        # true if objectA is a block
        pass

    @predicate(Object)
    def fixed(self, objectA):
        # true if objectA is fixed
        pass

    @predicate(Object, Object)
    def on(self, objectA, objectB):
        # true if objectA is on objectB
        pass

    @predicate(Object)
    def clear(self, objectA):
        # true if blockA can be grasped without knocking over other blocks i.e. blockA is on top
        pass

    @action(Object, Object, Object)
    def move(self, block, underObject, newUnderObject):
        # precondition is that block is of type Block
        # underObject is object currently underneath block (Location or Block)
        # newUnderObject is object desired to be underneath block
        precond = [~self.fixed(block), self.Block(block), self.on(block, underObject), self.clear(block),
                   self.clear(newUnderObject)]
        effect = [~self.on(block, underObject), self.on(block, newUnderObject), self.clear(block),
                  self.clear(underObject), ~self.clear(newUnderObject)]
        return precond, effect