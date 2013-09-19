#FrontEnd reference from Ground Truth
frontendreference100Hz = ThreeData()
frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.1.data', cov=True)
frontendreference100Hz.eigenValues()

#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.1.data', cov=True)
frontendbody100Hz.eigenValues()

#Back End Pose Out velocity
backendvelo100Hz = ThreeData()
backendvelo100Hz.readData('data/normal_spacehall/backend_poseout_velocity.1154.1.data', cov=True)
backendvelo100Hz.eigenValues()


#Back End Front End velocity comparisons
plt.figure(1)
values = frontendbody100Hz.getAxis(0)
plt.plot(frontendbody100Hz.time, values,
        marker='.', label="Motion Model X-axis", color=[1,0,1], lw=2)
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(0)
plt.plot(backendvelo100Hz.time, values,
        marker='.', label="Filter X-axis", color=[1,0.9,0], lw=2)
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(0)
plt.plot(frontendreference100Hz.time, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_x_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(1)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model Y-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(1)
plt.plot(backendvelo100Hz.t, values,
        marker='.', label="Filter Y-axis", color=[1,0.6,0], lw=2)
plt.plot(backendvelo100Hz.t, backendvelo100Hz.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.t, backendvelo100Hz.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(1)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_y_velocity_plot.png')


plt.figure(1)
values = frontendbody100Hz.getAxis(2)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model X-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(2, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(2, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(2)
plt.plot(backendvelo100Hz.t, values,
        marker='.', label="Filter Z-axis", color=[1,0.6,0], lw=2)
plt.plot(backendvelo100Hz.t, backendvelo100Hz.getStdMax(2, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.t, backendvelo100Hz.getStdMin(2, 3) , color=[0,0,0], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(2)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_z_velocity_plot.png')

