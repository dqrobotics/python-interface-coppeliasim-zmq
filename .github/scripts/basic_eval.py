from dqrobotics import *
from dqrobotics.interfaces.coppeliasim import *

def main():
    ci = DQ_CoppeliaSimInterfaceZMQ()
    print(ci)

if __name__ == "__main__":
    main()