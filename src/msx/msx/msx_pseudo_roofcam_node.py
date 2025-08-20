from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from termcolor import colored

class MSXPseudoRoofcamNode(DrivingSwarmNode):
    def __init__(self, name: str) -> None:
        super().__init__(name)


def main():
    main_fn('MSXPseudoRoofcamNode', MSXPseudoRoofcamNode)

if __name__ == '__main__':
    main()
