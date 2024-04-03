"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy observer, performs the state estimation parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import math

WHEEL_RADIUS = 0.0
WHEEL_AXIS   = 0.0


def init_pose (robot_pars):
    """
    Initialize the observer, using the given robot's parameters
    Return True if the inizialization was successful
    """
    WHEEL_RADIUS = robot_pars['wheel_radius']
    WHEEL_AXIS   = robot_pars['wheel_axis']
    return True


def update_pose (wl, wr):
    """
    Incremental position estimation: update the current pose (x, y, th) of the robot
    taking into account the newly read positions (rotations) of the left and right wheels
    Returns the new robot's pose, as a triple (x, y, th)
    """
    return (0.0, 0.0, 0.0)


class Occupancy ():
    def __init__(self, prior = (1.0, 0.0, 0.0, 0.0)):
        self.ukn  = prior[0]      # confidence that occupancy = unknown
        self.occ  = prior[1]      # confidence that occupancy = occupied
        self.ept  = prior[2]      # confidence that occupancy = empty
        self.cft  = prior[3]      # confidence that occupancy = conflict
    
    def __repr__(self):
        return "({:.2f} {:.2f} {:.2f} {:.2f})".format(self.ukn, self.occ, self.ept, self.cft)


class Gridmap ():
    def __init__(self, pars):
        self.grid = []                          # the grid (an array of Occupancy objects)
        self.nrows = 400                        # number of rows
        self.ncols = 300                        # number of columns
        self.xoff  = -2.0                       # x coordinate of bottom left cell
        self.yoff  = -35.0                      # y coordinate of bottom left cell
        self.cellsize = 0.1                     # cell size, in meters
        self.maxrange = pars['sonar_maxrange']  # cutoff sonar range 
        self.sdelta = pars['sonar_delta']       # sonar aperture angle
        self.lutable = []                       # lookup table of (rho, alpha)
        self.smodel = Sonar_Model()             # sonar sensor model

    def init_grid (self, offset = None, size = None):
        """
        Initialize the grid parameters
        Initialize the grid to an array of Occupacy objects
        Create the lookup table with the correct (rho,alpha) pairs
        """
        if offset:
            self.xoff = offset[0]
            self.yoff = offset[1]
        if size:
            self.nrows = size[0]
            self.ncols = size[1]
        self.grid = [[None] * self.ncols for i in range(self.nrows)]
        for i in range(self.nrows):
            for j in range(self.ncols):

                # initialize each cell to the priors of the chosen uncertainty theory
                self.grid[i][j] = Occupancy(self.smodel.prior())

        lusize = int(self.maxrange / self.cellsize) * 2 + 1
        self.lutable = [[None] * lusize for i in range(lusize)]
        for i in range(lusize):
            for j in range(lusize):
                dx = (j - lusize/2 + 1) * self.cellsize
                dy = (i - lusize/2 + 1) * self.cellsize
                rho = math.sqrt(dx*dx + dy*dy)
                alpha = math.atan2(dy, dx)
                self.lutable[i][j] = (rho, alpha)

    def x_to_col (self, x):
        """
        Return the column index corresponding to a given X metric coordinate
        """
        col = int((x - self.xoff) / self.cellsize)
        assert (col >= 0 and col < self.ncols), f"X value out of grid boundaries: {x}"
        return col

    def y_to_row (self, y):
        """
        Return the row index corresponding to a given Y metric coordinate
        """
        row = int((y - self.yoff) / self.cellsize)
        assert (row >= 0 and row < self.nrows), f"Y value out of grid boundaries: {y}"
        return row
    
    def col_to_x (self, j):
        """
        Return the X metric coordinate correspongind to a given column index
        """
        return self.xoff + j*self.cellsize

    def row_to_y (self, i):
        """
        Return the Y metric coordinate correspongind to a given row index
        """
        return self.yoff + i*self.cellsize

    def update_grid (self, sdata, mypose, debug = 1):
        """
        Top level function to update the grid by incorporating new sonar data
        Sdata is an array of sonar readings in the form (sonar pose, range)
        These are normally produced by get_sonar_data() in robot_gwy.py
        Mypose is the robot's pose (x, y, th) in the global reference frame
        """
        scansize = int(self.maxrange / self.cellsize)
        sinth = math.sin(mypose[2])
        costh = math.cos(mypose[2])
        newval = Occupancy()            # new estimated occupancy values
        for s in sdata:
            # scan all the received data from the sonar ring
            srange = s[1]
            if srange > self.maxrange:  # ignore out of range readings
                continue

            # compute the pose of the sonar s in the global frame
            spose = s[0]
            sx  = mypose[0] + spose[0] * costh - spose[1] * sinth
            sy  = mypose[1] + spose[0] * sinth + spose[1] * costh
            sth = mypose[2] + spose[2]
            if debug > 1:
                print("Fusing sonar at ({:.3f}, {:.3f}, {:.0f}) (range = {:.3f})".format(sx, sy, math.degrees(sth), srange))

            # compute the origin of the area that we should update
            start_i = self.y_to_row(sy) - scansize
            start_j = self.x_to_col(sx) - scansize

            # scan all cells in the update area, see if we should update them
            for i in range(scansize * 2 + 1):
                ci = start_i + i
                if ci < 0 or ci >= self.nrows:
                    continue
                for j in range(scansize * 2 + 1):
                    cj = start_j + j
                    if cj < 0 or cj >= self.ncols:
                        continue

                    # we have found a cell (i,j) that should be updated
                    # compute its (rho,phi)
                    rho   = self.lutable[i][j][0]
                    alpha = self.lutable[i][j][1]
                    phi = alpha - sth

                    if rho > self.maxrange:  # ignore cells beyond the max range
                        continue

                    # working with angle differences is tricky, make sure to normalize
                    if phi > math.pi:
                        phi -= 2.0 * math.pi
                    elif phi < -math.pi:
                        phi += 2.0 * math.pi

                    # ask the sensor model what's the support for this cell
                    self.smodel.support(rho, phi, self.sdelta, srange, newval)
                    if debug > 2:
                        print("Scanning cell ({},{}) at ({:.3f}, {:.3f})".format(ci, cj, self.col_to_x(cj), self.row_to_y(ci)))
                        print("  with (rho,alpha) = ({:.3f},{:.0f}), phi = {:.0f}".format(rho,math.degrees(alpha),math.degrees(phi)))
                        print("  support:", newval)
                        print("")
                    
                    # fuse this support with the previous values in this cell
                    self.smodel.fuse(self.grid[ci][cj], newval)

    def find_minmax (self, layer = 'occ'):
        """
        Find the min and max values in a given layer of the gridmap
        """
        minval = 0.0
        maxval = 0.0
        for row in self.grid:
            for c in row:
                if getattr(c,layer) < minval:
                    minval = getattr(c,layer)
                if getattr(c,layer) > maxval:
                    maxval = getattr(c,layer)
        return minval, maxval

    def print_grid (self, filename = None, layer = 'occ'):
        """
        Save a given layer of the gridmap as a PGM image file, for visualization
        """
        if filename == None:
            filename = layer + ".pgm"
        minval, maxval = self.find_minmax(layer = layer)
        levels = max(1, int(maxval - minval))
        with open(filename, 'w') as file:
            file.write("P2\n{} {} {}\n".format(self.ncols, self.nrows, levels))
            for i in range(self.nrows):
                for c in self.grid[self.nrows-i-1]:
                    file.write("{} ".format(int(getattr(c,layer) - minval)))
    
    def close_grid (self):
        pass


class Sonar_Model ():
    """
    Compute the amount of support for each hypothesis about a cell at (rho, phi)
    given a reading with range r from a sonar with aperture delta
    How the amount of support is encoded depends on the uncertainty theory used
    """
    def __init__(self):
        pass

    def prior (self):
        """
        Initial values for the {ukn, occ, ept, cft} hypotheses depend on the uncertainty theory used
        Here we use a trivial 'hit-count' method
        """
        return (1.0, 0.0, 0.0, 0.0)

    def support (self, rho, phi, delta, r, result):
        """
        Actual values for the {ukn, occ, ept, cft} hypotheses depend on the uncertainty theory used
        Here we use a trivial 'hit-count' method
        """
        # check that cell at (rho, phi) is in the field of view
        if (phi <= delta/2 and -phi <= delta/2):
            if abs(rho - r) <= 0.02:
                # cell is at the measured range: support the 'occ' hypothesis
                result.ukn = 0.0
                result.occ = 1.0
                result.ept = 0.0
                result.cft = 0.0
            elif r < rho:
                # cell is before measured range: support the 'ept' hypothesis
                result.ukn = 0.0
                result.occ = 0.0
                result.ept = 1.0
                result.cft = 0.0
            else:
                # cell is beyond measured area: support the 'ukn' hypothesis
                result.ukn = 1.0
                result.occ = 0.0
                result.ept = 0.0
                result.cft = 0.0
        else:
            # cell is not in the field of view: support the 'ukn' hypothesis
            result.ukn = 1.0
            result.occ = 0.0
            result.ept = 0.0
            result.cft = 0.0
        return result

    def fuse (self, old, new):
        """
        Fuse the new values for (ukn, occ, ept, cft) with the previous ones
        Return the fused values
        Fusion details depend on the uncertainty theory,
        here we use a trivial count of the 'occ', 'ept' hits
        """
        old.ukn = min(old.ukn, new.ukn)
        old.occ = max(old.occ, new.occ)
        if old.occ < 0.0:
            old.occ = 0.0
        old.ept = max(old.ept, new.ept)
        if old.ept < 0.0:
            old.ept = 0.0
        old.cft = 0.0
        return old

