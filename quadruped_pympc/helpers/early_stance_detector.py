import numpy as np
from quadruped_pympc import config as cfg
from gym_quadruped.utils.quadruped_utils import LegsAttr
EARLY_STANCE_THRESHOLD = 0.05
class EarlyStanceDetector:
    def __init__(self, feet_geom_id,
                 legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR')):
        self.legs_order = legs_order
        self.feet_geom_id = feet_geom_id
        self.early_stance = LegsAttr( FL=False, FR=False, RR=False, RL=False )
        self.hitmoments = LegsAttr(FL=-1.0, FR=-1.0, RR=-1.0, RL=-1.0) # swing time of the last contact moment
        self.hitpoints = LegsAttr(FL=None, FR=None, RR=None, RL=None)
        self.contact = None

    def update(self,contact,feet_pos:LegsAttr,lift_off:LegsAttr, touch_down:LegsAttr, swing_time:list, swing_period:float):
        """ 
        Update the early stance detector.
        
        Parameters:
            contact : mjContact List of all contacts.
            lift_off_positions : Lift off positions of all legs.
            touch_down_positions : Touch down positions of all legs.
        """
        self.contact = contact
        for leg_id,leg_name in enumerate(self.legs_order):
            contact_points = self.contact_points(leg_name)
            disp = touch_down[leg_name] - lift_off[leg_name]
            for contact_point in contact_points:
                # if np.linalg.norm(contact_point - touch_down[leg_name]) < EARLY_STANCE_THRESHOLD or np.linalg.norm(contact_point - lift_off[leg_name]) < EARLY_STANCE_THRESHOLD:
                if swing_time[leg_id] < EARLY_STANCE_THRESHOLD or swing_time[leg_id] > swing_period - EARLY_STANCE_THRESHOLD:
                    self.early_stance[leg_name] = False  # reset early stance if contact point is close to touch down position or lift off position
                    break
                else:
                    local_disp = (contact_point - feet_pos[leg_name]).squeeze()
                    if self.early_stance[leg_name] == False:
                        if np.arccos(np.dot(disp, local_disp) / (np.linalg.norm(disp) * np.linalg.norm(local_disp))) < np.pi/3: 
                            self.hitpoints[leg_name] = contact_point.copy()
                            self.hitmoments[leg_name] = swing_time[leg_id]
                            self.early_stance[leg_name] = True  # acos( disp dot local_disp / |disp| |local_disp|) < 60°
                            break
                        else:
                            self.early_stance[leg_name] = False
                            self.hitmoments[leg_name] = -1.0
                            self.hitpoints[leg_name] = None
            if self.early_stance[leg_name] != True:
                self.hitmoments[leg_name] = -1.0
                self.hitpoints[leg_name] = None

    def contact_points(self, leg_name):
        """
        Get the contact points of a leg. Return None if the leg is not in contact.
        
        Parameters:
            leg_name : str Name of the leg.
        
        Returns:
            contact_points : list Contact points of the leg.
        """
        contact_points = []
        contact_id = np.where(np.any(self.contact.geom == self.feet_geom_id[leg_name],axis=1))
        # check contact_id is not empty
        if contact_id[0].size > 0:
            for i in range(contact_id[0].size):
                contact_points.append(self.contact.pos[contact_id[0][i]])
        return contact_points