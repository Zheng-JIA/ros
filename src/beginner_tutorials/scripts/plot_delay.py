import cPickle
import numpy
import matplotlib.pyplot as plt
import matplotlib.legend as legend

def plot_delay():
    f = file("/home/zheng/ros/src/beginner_tutorials/scripts/delay",'rb')
    delay_s = cPickle.load(f)
    delay_ms = [delay*1000 for delay in delay_s]
    f.close()
    plt.hist(delay_ms, bins=20)
    plt.show()
if __name__=='__main__':
    plot_delay()
