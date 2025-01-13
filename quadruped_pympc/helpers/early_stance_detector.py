from quadruped_pympc import config as cfg
from gym_quadruped.utils.quadruped_utils import LegsAttr

class EarlyStanceDetector:
    def __init__(self, feet_geom_id):
        self.feet_geom_id = feet_geom_id
        self.early_stance = LegsAttr( FL=False, FR=False, RR=False, RL=False )


    def update(self, movement_direction,contact,lift_off:LegsAttr, touch_down:LegsAttr):
        """ 
        Update the early stance detector.
        
        Parameters:
            movement_direction : The direction of the movement.
            contact : mjContact List of all contacts.
            lift_off_positions : Lift off positions of all legs.
            touch_down_positions : Touch down positions of all legs.
        """
        # Get the contact points of the feet
        contact_points = []
        ...