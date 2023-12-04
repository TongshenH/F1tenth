class MaxGapState(object):

    def __init__(self):
        self.max_gap_min = 0
        self.max_gap_max = 0
        self.ranges = None

    def update(self, data):
        self.max_gap_min = data.angle_min
        self.max_gap_max = data.angle_max
        self.ranges = data.ranges

    def get_update(self):
        return self.max_gap_min, self.max_gap_max, self.ranges