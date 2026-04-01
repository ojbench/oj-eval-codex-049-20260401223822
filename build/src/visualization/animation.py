from matplotlib import animation, pyplot as plt, patches

import config
from file import file_reader


class Animation:
    def __init__(self, file_name):
        self.fig, self.ax = plt.subplots()
        self.ani = None
        self.file_reader = file_reader()
        self.file_reader.read_info(file_name)
        self.beg_round = self.file_reader.beg_round

    def update(self, frame):  # update function for drawing each frame
        artists = []
        self.ax.clear()
        for i in range(self.file_reader.robot_num):
            artists.extend(self.ax.plot(self.file_reader.x_trace[i][:frame + self.beg_round], self.file_reader.y_trace[i][:frame + self.beg_round], linewidth=0.8))
            artists.append(self.ax.add_patch(patches.Circle((self.file_reader.x_trace[i][frame + self.beg_round - 1], self.file_reader.y_trace[i][frame + self.beg_round - 1]),
                                                            radius=self.file_reader.r[i], edgecolor='blue', facecolor='orange', alpha=0.6)))
            artists.append(self.ax.scatter(self.file_reader.x_trace[i][frame + self.beg_round - 1], self.file_reader.y_trace[i][frame + self.beg_round - 1], s=5))
            artists.append(self.ax.scatter(self.file_reader.x_tar[i], self.file_reader.y_tar[i], s=5))
            artists.append(self.ax.arrow(self.file_reader.x_trace[i][frame + self.beg_round - 1], self.file_reader.y_trace[i][frame + self.beg_round - 1],
                                         (self.file_reader.x_tar[i] - self.file_reader.x_trace[i][frame + self.beg_round - 1]) * 0.98,
                                         (self.file_reader.y_tar[i] - self.file_reader.y_trace[i][frame + self.beg_round - 1]) * 0.98,
                                         head_width=0.2, head_length=0.2, fc='red', ec='black', linestyle=(0, (5, 5)), linewidth=0.6))
        self.ax.set_xlim(self.file_reader.x_min - 5, self.file_reader.x_max + 5)
        self.ax.set_ylim(self.file_reader.y_min - 0.5, self.file_reader.y_max + 0.5)
        self.ax.set_title('Round {}'.format((frame + self.beg_round) * config.interval))

        plt.gca().set_aspect('equal')
        return artists

    def save(self):
        plt.close('all')
        self.fig, self.ax = plt.subplots()
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=int((self.file_reader.end_round - self.file_reader.beg_round) / config.interval), interval=2)
        self.ani.save(config.gif_name, writer='pillow', fps=config.fps)
