# Plotting and analysis utilities
# Author: Tyler
# Data: May 12, 2019

from util import *

def analyze(fname):
    make_data_dir()
    if fname == "latest":
        files = glob.glob(os.path.join(cwd, 'data' + os.sep + '*.dat'))
        fname = max(files, key=os.path.getctime)

    SAMPLE_RATE = 100.0 # Hz
    angles, num_samples = load_angles_from_file(fname)
    t = np.linspace(0, num_samples / SAMPLE_RATE, num=num_samples, endpoint=False)

    fig, ax = plt.subplots()
    size = 2
    
    ax.scatter(t, angles[BASE_OUTER], c="blue",  label="Base (outer)", s=size)
    ax.scatter(t, angles[BASE_INNER], c="red",   label="Base (inner)", s=size)
    ax.scatter(t, angles[LAMP_OUTER], c="green", label="Lamp (outer)", s=size)
    ax.scatter(t, angles[LAMP_INNER], c="black", label="Lamp (inner)", s=size)
    ax.legend()
    
    plt.title('Angles vs time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle ($^\circ$)')
    
    fig_name = os.path.join(cwd, os.path.splitext(fname)[0] + '.png')
    plt.savefig(fig_name)
    logString("Saved fig to {0}".format(fig_name))
    plt.close();