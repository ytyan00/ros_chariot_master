import hoyer_sling_base
import hoyer_sling_lift
import hoyer_sling_frame

class hoyer_sling():
    def __init__(self) -> None:
        self.base = hoyer_sling_base.hoyer_sling_base()
        self.lift = hoyer_sling_lift.hoyer_sling_lift()
        self.frame = hoyer_sling_frame.hoyer_sling_frame()
        