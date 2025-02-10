import sys
import os
# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Add the parent directory of src to sys.path   
from src.MI_run import *




if __name__ == "__main__":
    main()
