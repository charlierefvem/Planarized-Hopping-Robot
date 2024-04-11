# inspired by http://nipunbatra.github.io/2014/08/latexify/
params = {
    'image.origin': 'lower',
    'image.interpolation': 'nearest',
    'image.cmap': 'gray',
    'axes.grid': False,
    'savefig.dpi': 150,  # to adjust notebook inline plot size
    'axes.labelsize': 14, # fontsize for x and y labels (was 10)
    'axes.titlesize': 16,
    'font.size': 14, # was 10
    'legend.fontsize': 14, # was 10
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'text.usetex': True,
    'figure.figsize': [3.39, 2.10],
    'font.family': 'serif',
}
import matplotlib
matplotlib.rcParams.update(params)
 
def myPlotter(loss_arr, file_name):
    # validation data
    res_multiplier = 8
  
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    import seaborn as sns
 
    mpl.rcParams['pdf.fonttype'] = 42
    mpl.rcParams['font.size'] = 14
 
    color_scheme = sns.color_palette("rocket", as_cmap=True)
 
    fig = plt.figure(1, figsize=(9,6))
    gs = gridspec.GridSpec(9, 6,
                       #hspace=0.8,  # horizontal space
                       wspace=0.5)
    #gs.update (width=800, height=600)  
 
    spatial_mesh_arr = [tf.cast(tf.linspace(lb,ub, N), dtype='float32')  for (lb, ub, N) in
                zip(domain_lb, domain_ub, [500, 500])]
 
    mesh_grid = tf.meshgrid(*spatial_mesh_arr)
 
    #use fixed y, varying y
    grid_coord = [tf.reshape(x,(-1,1)) for x in mesh_grid]
    spatial_coord = tf.squeeze(tf.stack(grid_coord, axis=-1))
 
    # t=0 initial condition
    ax2 = fig.add_subplot(gs[0:3, 0:3])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
    x = spatial_coord[:,0]
    y = spatial_coord[:,1]
    x = tf.reshape(x, (x.shape[0],-1))
    y = tf.reshape(y, (y.shape[0],-1))
    t = tf.zeros_like(x) # t=0
    coord = tf.concat([t, x, y], axis=-1)
    pred = model(coord)
    pred = tf.reshape(pred, mesh_grid[0].shape)
    anal = anal_func(t,x,y)
    anal = tf.reshape(anal, mesh_grid[0].shape)
    import matplotlib.colors as mcolors
    norm = mcolors.Normalize(vmin=0.0, vmax=1.0)  # Adjust color limits
 
    subfig2 = ax2.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), pred, cmap= color_scheme,  norm=norm)
 
  
    ax2.set_xlim([-10,30])
    ax2.set_ylim([-10,10])
    ax2.xaxis.set_visible(False)
    ax2.yaxis.set_visible(False)
 
    ax5 = fig.add_subplot(gs[0:3, 3:6])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
    subfig5 = ax5.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), anal.numpy(), cmap= color_scheme, norm=norm)
    ax5.xaxis.set_visible(False)
    ax5.yaxis.set_visible(False)
    ax5.set_xlim([-10,30])
    ax5.set_ylim([-10,10])
 
    #t=t_max/2
    ax3 = fig.add_subplot(gs[3:6, 0:3])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
  
    t = t_max/2 * tf.ones_like(x)
    coord = tf.concat([t, x, y],axis=-1)
    pred = model(coord)
    pred = tf.reshape(pred, mesh_grid[0].shape)
    anal = anal_func(t,x,y)
    anal = tf.reshape(anal, mesh_grid[0].shape)
 
    subfig3 = ax3.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), pred, cmap= color_scheme,  norm=norm)
 
    ax3.set_xlabel(r'$x$')
    ax3.set_ylabel(r'$y$')
    #cbar3 = fig.colorbar(subfig3) # Add a colorbar to a plot
    ax3.set_xlim([-10,30])
    ax3.set_ylim([-10,10])
    ax3.xaxis.set_visible(False)
    #ax3.yaxis.set_visible(False)

    ax6 = fig.add_subplot(gs[3:6, 3:6])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
    subfig6 = ax6.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), anal.numpy(), cmap= color_scheme,  norm=norm)
    ax6.set_xlim([-10,30])
    ax6.set_ylim([-10,10])
    ax6.xaxis.set_visible(False)
    ax6.yaxis.set_visible(False)
  
 
    #t=t_max
    ax4 = fig.add_subplot(gs[6:9, 0:3])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
    t = t_max * tf.ones_like(x)
    coord = tf.concat([t, x, y],axis=-1)
    pred = model(coord)
    pred = tf.reshape(pred, mesh_grid[0].shape)
    anal = anal_func(t,x,y)
    anal = tf.reshape(anal, mesh_grid[0].shape)
 
    subfig4 = ax4.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), pred, cmap= color_scheme,  norm=norm)
    ax4.yaxis.set_visible(False)
 
    ax4.set_xlabel(r'$x$')
    ax4.set_xlim([-10,30])
    ax4.set_ylim([-10,10])
 
    ax7 = fig.add_subplot(gs[6:9, 3:6])
    cm = mpl.colormaps.get_cmap('RdYlBu')
 
    subfig7 = ax7.pcolor(mesh_grid[0].numpy(), mesh_grid[1].numpy(), anal.numpy(), cmap= color_scheme,  norm=norm)
    ax7.yaxis.set_visible(False)
    ax7.set_xlim([-10,30])
    ax7.set_ylim([-10,10])
    ax7.set_xlabel(r'$x$')
 
    #plt.legend(handles=[line1,  line3], loc="lower right")
    fig.subplots_adjust(right=0.90)
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    cbar = fig.colorbar(subfig3, cax=cbar_ax)
 
    min_val = tf.reduce_min(pred)
    max_val = tf.reduce_max(pred)
    import numpy as np
    ticks = np.linspace(min_val, max_val, 5)
    cbar.set_ticks([0, 0.25, 0.75, 1.0])
    #cbar.set_ticklabels(['{:.1f}'.format(t) for t in ticks])
    #plt.show()
 
    #plt.tight_layout(h_pad=1)
    plt.savefig(file_name, format='png')
    plt.clf()  # Clear the entire figure