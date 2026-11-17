class CavData:
    def __init__(
            self,
            predicted_state,
            loc,
            spd,
            ff_spd=None,
            current_ref_file=None,
            current_intersection=None,
            current_approach=None,
            sec_to_change=None,
            signal_status=None,
            spat_intersection_id=None,
            spat_rx_time=None,
            expected_signal_id=None):
        self.predicted_state = predicted_state
        self.loc = loc
        self.spd = spd
        self.ff_spd = ff_spd
        self.current_ref_file = current_ref_file
        self.current_intersection = current_intersection
        self.current_approach = current_approach
        # ESSCV: SPaT time to the next change of the current phase (minEndTime, s) and its state
        # ("green"/"yellow"/"red"); None when no SPaT for the selected signal group has been decoded
        self.sec_to_change = sec_to_change
        self.signal_status = signal_status
        # ESSCV: where/when that SPaT came from, so stale SPaT or SPaT from another intersection is not used:
        # intersection id in the SPaT, wall-clock time it was decoded (time.time()), and the intersection id
        # the current reference expects (signal_group_ids in distance_finder.py)
        self.spat_intersection_id = spat_intersection_id
        self.spat_rx_time = spat_rx_time
        self.expected_signal_id = expected_signal_id

class PredData:
    def __init__(
            self,
            pos_pred_ego,
            pos_pred_max_ego,
            pos_pred_min_ego,
            spd_pred_ego,
            predicted_tl_state,
            pos_ego,
            spd_ego,
            current_ref_file=None,
            current_intersection=None,
            current_approach=None):
        self.pos_pred_ego = pos_pred_ego
        self.pos_pred_max_ego = pos_pred_max_ego
        self.pos_pred_min_ego = pos_pred_min_ego
        self.spd_pred_ego = spd_pred_ego
        self.predicted_tl_state = predicted_tl_state
        self.pos_ego = pos_ego
        self.spd_ego = spd_ego
        self.current_ref_file = current_ref_file
        self.current_intersection = current_intersection
        self.current_approach = current_approach
