import csv

class Env:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.gap = 0.05
        # self.obs_boundary = self.obs_boundary()
        # self.obs_rectangle = self.obs_rectangle()
        self.file = open('../map.csv')
        self.map, self.x_range, self.y_range = self.get_map()
        self.obs = self.get_obs()

    def get_map(self):
        csvreader = csv.reader(self.file)
        map = []
        for row in csvreader:
            map.append(row)
        self.height = len(map)
        self.width = len(map[0])
        x_range = (-self.width / 2, self.width / 2) 
        y_range = (-self.height / 2, self.height / 2)

        return map, x_range, y_range

    def get_obs(self):
        obs = [] 
        for i in range(self.height):
            for j in range(self.width):
                if self.map[i][j] == '100':
                    obs.append([self.width/2 - j, self.height/2 -i])

        return obs


