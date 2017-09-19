import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from numpy.linalg import inv

import pydb
import config

import logging

class Fast_localize:

    def __init__(self, places, Z, dop_db, dx_db):
        self.places = places
        self.res = 0.05
        self.moves = len(places)
        self.Z = Z
        self.dop_db = dop_db
        self.dx_db = dx_db

    def locate_aps(self):
        waits = 4
        threshold = 150.0
        location = {}
        x = np.zeros((4,1), dtype=np.float32)
        print 'number of points explored:',self.moves
        if self.moves > waits:
            for each in self.Z.keys():
                # pydb.debugger()
                ind_tbd = self.get_filter(self.Z[each])
                # self.log('ind: \n{0}'.format(ind_tbd))
                RSS = self.apply_filter_RSS(ind_tbd, each)
                # self.log('RSS: \n{0}'.format(RSS))
                places = self.apply_filter_places(ind_tbd)
                if len(places) < waits:
                    continue
                mean = np.mean(places, axis=0, dtype=np.float32)
                x[0]=mean[0]; x[1]=mean[1]; x[2]=3.0; x[3]=34.0
                R = np.zeros((len(places), len(places)), dtype=np.float32)
                np.fill_diagonal(R, 36)
                mod_dx = 1.0
                # self.log('RSS and AP: \n{0} {1}'.format(RSS, each))
                loop = 0
                while mod_dx < threshold:
                    loop = loop + 1
                    dist = self.get_dist(x, places)
                    H = self.get_H(x, dist, places)
                    hx = self.get_hx(x, places)
                    delta_z = self.get_delta_z(RSS,hx)
                    covar_dx = inv(np.dot(np.dot(H, inv(R)), np.transpose(H)))
                    delta_x = np.dot(np.dot(np.dot(covar_dx, H),inv(R)),delta_z)
                    mod_dx = sqrt(delta_x[0][0]**2 + delta_x[1][0]**2 + delta_x[2][0]**2 + delta_x[3][0]**2)
                    # self.log('mod_dx: {0}'.format(mod_dx))
                    if mod_dx > threshold or mod_dx < 0.01 or loop > 200:
                        break
                    newx = x + delta_x
                    # pydb.debugger()
                    # self.log('delta_z: \n{0}'.format(delta_z))
                    # self.log('covar_dx \n{0}'.format(covar_dx))
                    # self.log('delta_x \n{0}'.format(delta_x))
                    # self.log('newx: \n{0}'.format(newx))
                    prev_mod_dx = self.prev_mod_dx(mod_dx, each)
                    dop = self.get_dop(H,R)
                    # self.log('DOP: {0}'.format(dop))
                    prev_dop = self.prev_dop(dop, each)
                    range = sqrt((newx[0]-places[-1][0])**2 + (newx[1]-places[-1][1])**2)
                    if (newx[3]<101) and range<200 and dop < prev_dop and mod_dx < prev_mod_dx:
                        self.update_dop_db(dop, each)
                        self.update_dx_db(mod_dx, each)
                        if not location.keys():
                            location.update({each:(newx[0][0],newx[1][0])})
                        else:
                            location[each] = (newx[0][0],newx[1][0])
                    # self.log('est: \n{0}'.format(location))
                    x = newx
                    # self.log('dop_db: \n{0}'.format(self.dop_db))
                    # self.log('dx_db: \n{0}'.format(self.dx_db))
            # self.log('est: \n{0}'.format(location))
            if location:
                error = self.get_error(location)
                self.log('Error: \n{0}'.format(error))
        return location

    def prev_mod_dx(self, mod_dx, key):
        if key in self.dx_db.keys():
            prev_mod_dx = self.dx_db[key]
        else:
            prev_mod_dx = mod_dx + 1
        return prev_mod_dx

    def prev_dop(self, dop, key):
        if key in self.dop_db.keys():
            prev_dop = self.dop_db[key]
        else:
            prev_dop = dop + 1
        return prev_dop

    def get_delta_z(self, RSS, hx):
        result = np.empty((hx.shape[0],1), dtype=np.float32)
        for i in range(hx.shape[0]):
            result[i] = RSS[i] - hx[i]
        return result

    def get_error(self, location):
        error = {}
        for key in location.keys():
            if key not in error.keys() or not error.keys():
                error.update({key:sqrt((location[key][0] - config.load_env['x'][key-1])**2 + (location[key][1] - config.load_env['y'][key-1])**2)})
            else:
                error[key] = sqrt((location[key][0] - config.load_env['x'][key-1]) ** 2 + (location[key][1] - config.load_env['y'][key-1])** 2)
        return error

    def apply_filter_RSS(self, ind_tbd, index):
        result = []
        for i in range(len(self.Z[index])):
            if i in ind_tbd:
                continue
            result.append(self.Z[index][i])
        return result

    def apply_filter_places(self, ind_tbd):
        result = []
        for i in range(len(self.places)):
            if i in ind_tbd:
                continue
            result.append(self.places[i])
        return result

    def get_filter(self, array):
        result = []
        for i in range(len(array)-1):
            if array[i] == 0:
                result.append(i)
        return result

    def update_dop_db(self, curr_dop, key):
        if key not in self.dop_db.keys() or not self.dop_db.keys():
            self.dop_db.update({key : curr_dop})
        else:
            self.dop_db[key] = curr_dop

    def update_dx_db(self, curr_dx, key):
        if key not in self.dx_db.keys() or not self.dx_db.keys():
            self.dx_db.update({key : curr_dx})
        else:
            self.dx_db[key] = curr_dx

    def get_dop(self, H, R):
        # pydb.debugger()
        return sqrt(np.trace(inv(np.dot(np.dot(H, inv(R)), np.transpose(H)))))

    def get_hx(self, x, places):
        xo = x[0]
        yo = x[1]
        n = x[2]
        A = x[3]
        hx = np.zeros((len(places),1), dtype=np.float32)
        for i in range(len(places)):
            hx[i] = (-10)*n*np.log10(sqrt((xo-places[i][0])**2+(yo-places[i][1])**2)) - A
        return hx

    def get_H(self, x, dist, places):
        xo = x[0]
        yo = x[1]
        n = x[2]
        l = np.log(10)
        H = np.zeros((4,len(dist)), dtype=np.float32)
        for i in range(H.shape[0]):
            for j in range(H.shape[1]):
                if i in [0]: H[i, j] = -10 * n * (xo - places[j][0]) / (l * (dist[j]** 2))
                if i in [1]: H[i, j] = -10 * n * (yo - places[j][1]) / (l * (dist[j]** 2))
                if i in [2]: H[i, j] = -10 * np.log10(dist[j])
                if i in [3]: H[i, j] = -1
        return H

    def get_dist(self, x, places):
        result = []
        for i in range(len(places)):
            result.append(sqrt((places[i][0]-x[0])**2+(places[i][1]-x[1])**2))
        return result

    def log(self, message):
        rospy.loginfo(message)

    def plot_set(self, set):
        if len(set) > 1:
            plt.plot(set[0],set[1],'r--',set[2],set[3],'g^',)
        else:
            plt.plot(set[0],set[1],'b')
        plt.show()

        # R = np.zeros(Qr.shape, dtype=np.float32)
        # for i in range(Qr.shape[0]): R[i, i] = Qr[i, i]
        # for i in range(R.shape[0]): sum = sum + R[i, i]
        # R = R / sum
        # R = R * Qr
        # print 'Z:',self.Z