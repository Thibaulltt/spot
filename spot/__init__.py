"""
A set of wrappers around the SPOT method : from Sliced Partial Optimal Transport by Bonneel and Coeurjolly (2019).

For more information about the signature and innerworkings of the functions and classes defined within, a look at the
source code is more than helpful : https://github.com/thibaulltt/spot

All useful classes of this module are defined in a submodule named "_spot", imported here. It has been compiled from C++.
"""

# Import compiled module from C++ :
from ._spot import *

# Force set the version information :
__version__ = _spot.__version__
