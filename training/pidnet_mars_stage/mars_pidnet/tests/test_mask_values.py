import numpy as np
from mars_pidnet.datasets.mars_dataset import ALLOWED_FINE, ALLOWED_COARSE, FINE_TO_COARSE

def test_class_sets_and_mapping():
    assert ALLOWED_FINE==set(range(8))|{255}; assert ALLOWED_COARSE==set(range(6))|{255}
    x=np.array([0,1,2,3,4,5,6,7,255],np.uint8)
    assert FINE_TO_COARSE[x].tolist()==[0,1,2,3,4,5,5,5,255]
