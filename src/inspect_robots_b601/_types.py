"""Array types at the Inspect observation and action boundary."""

import numpy as np
from numpy.typing import NDArray

JointVector = NDArray[np.float64]
RGBImage = NDArray[np.uint8]
RosObservation = tuple[JointVector, float, dict[str, RGBImage], dict[str, float]]
