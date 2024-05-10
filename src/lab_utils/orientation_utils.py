import numpy as np
from scipy.spatial.transform import Rotation as R

def get_distance(x1, y1, x2, y2):
	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_angle(x1, y1, x2, y2):
	return np.degrees(np.arctan2(y2 - y1, x2 - x1))
    
# Fonction pour le calcul de l'orientation à partir d'un quaternion
# Entrée : Quaternion [x, y ,z ,w]
# Sortie : Angle de lacet (yaw ou heading) en DEGRES
def get_heading_from_quaternion(q):
    if (np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)) == 0:
        return 0
    else:
        r = R.from_quat([q.x, q.y, q.z, q.w])
        angles = r.as_euler('xyz', degrees=True) #to return angles in degrees
        return angles[2]
    
def get_euleurmatrix_from_quaternion(x, y, z, w):
    if (np.sqrt(x**2 + y**2 + z**2 + w**2)) == 0:
        return 0
    else:
        q = np.array([x, y, z, w])
        r = R.from_quat(q)
        #        angle = r.as_euler('xyz', degrees=True) #to return angles in degrees
        
        # Use as_dcm() for older versions of SciPy
        try:
            rotation_matrix = r.as_matrix()  # Newer versions
        except AttributeError:
            rotation_matrix = r.as_dcm()  # Older versions
        
        return rotation_matrix
