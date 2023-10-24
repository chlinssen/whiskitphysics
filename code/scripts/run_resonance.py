import matplotlib.pyplot as plt
import subprocess
import numpy as np



def generate_whisker_ang_vel(ramp_up_time: float, freq: float, ampl: float = 1, sim_time: float = .1, dt: float = 1E-3):
	N = int(np.ceil(sim_time / dt))
	timevec = np.linspace(0, sim_time, N)

	ang_vel = np.zeros((N, 3))
	ang_vel[:, 2] = ampl * np.sin(2 * np.pi * freq * np.linspace(0., sim_time, N))
	ang_vel[:, 2] *= np.minimum(np.ones_like(timevec), timevec / ramp_up_time)

	return ang_vel


def run_whiskit():
	cmd_line_list = "/home/charl/neuroinfo/whiskitphysics-fork-yaml_parameters/code/scripts/run_resonance.sh"
	proc = subprocess.Popen(cmd_line_list)
	proc.wait()
	if proc.returncode > 0:
		raise Exception('Error executing external tool! Return code ' + str(proc.returncode))


sim_time = .1   # [s]
dt = 1E-3   # [s]

ramp_up_time = 100E-3	# [s]
t_analysis_start = ramp_up_time * 3	# [s]

fmin = 10.	# [Hz]
fmax = 50.		# [Hz]
n_frequency_points = 12



def write_config(dt: float, sim_time: float, n_links: int):
	file_path_in = 'scripts/parameters/whisker_resonance.jinja2'
	file_path_out = 'scripts/parameters/whisker_resonance.yaml'

	with open(file_path_in, 'r') as file_in:
		content = file_in.read()
		modified_content = content.replace("{{ dt }}", str(dt))
		modified_content = modified_content.replace("{{ simulation_time_stop }}", str(sim_time))
		modified_content = modified_content.replace("{{ n_links }}", str(n_links))
		with open(file_path_out, 'w') as file_out:
			file_out.write(modified_content)



def read_data(fn: str, n_segments: int):
	data = np.genfromtxt(fn, delimiter=',')   # n_datapoints * n_segments
	print(data.shape)
	ampl_base = data[:, 0]
	ampl_tip = data[:, n_segments - 1]
	# plt.figure()
	# plt.imshow(data)
	# plt.show()


	return ampl_base, ampl_tip


def plot_timeseries(timevec, tidx_analysis_start, tidx_analysis_stop, x_base, x_tip, ampl_tip_mag, fn):
	plt.clf()
	fig = plt.figure(figsize=(12, 4), dpi=300)
	fig.suptitle('Simulation output timeseries and analysis window')
	ax = fig.add_axes([.1, .1, .8, .6])
	ax.plot(timevec, x_base, label="base", color="blue")
	ax.plot([timevec[tidx_analysis_start], timevec[tidx_analysis_start]], ax.get_ylim(), linewidth=3, color="green")
	ax.plot([timevec[tidx_analysis_stop], timevec[tidx_analysis_stop]], ax.get_ylim(), linewidth=3, color="green")

	ax2 = ax.twinx()
	ax2.plot(timevec, x_tip, label="tip", color="red")
	ax2.plot([0., np.amax(timevec)], 2 * [ampl_tip_mag], linewidth=2, color="grey")

	for _ax in [ax, ax2]:
		_ax.set_xlim(0., np.amax(timevec))
		_ax.grid(True)

	plt.savefig(fn)
	plt.close(fig)


def plot_resonance_curve(freq, base_ampl, ampl,  fname_snip="", ax_ampl=None, ax_phase=None, label=None, colour=None, freq_peak=None, ampl_peak=None):
	if colour is None:
		colour = "blue"
	fig = None
	if not ax_ampl:
		fig = plt.figure(figsize=(12,4))
		ax_ampl = fig.add_axes([.1, .3, .8, .6])
		ax_phase = fig.add_axes([.1, .15, .8, .1])
	# ax_ampl.loglog(freq, base_ampl, 'o-', label='base', linewidth=2., color='b')
	ax_ampl.loglog(freq, ampl, '-o', label=label, linewidth=2., color=colour)
	if freq_peak is not None and ampl_peak is not None:
		ax_ampl.loglog(freq_peak, ampl_peak, marker='o', markersize=20, c="red")
	# ax_ampl.set_ylim(ax_ampl.get_ylim()[0], 2. * np.nanmax(ampl))
	# ax_ampl.loglog([f_n * np.sqrt(1 - damping_ratio**2), f_n * np.sqrt(1 - damping_ratio**2)], [ax_ampl.get_ylim()[0], ax_ampl.get_ylim()[1]], '--', label='theory', linewidth=1., color='grey')
	ax_ampl.set_ylabel('Deflection [rad]')
	ax_ampl.set_xlim(freq[-1], freq[0])
	ax_ampl.grid(True, which="both")
	fig.savefig("/tmp/freq_response" + fname_snip + ".png", dpi=300)
	# ax_ampl.legend()

	# ax_phase.semilogx(freq, phase, '-', label=label, linewidth=2., color=colour)
	# ax_phase.set_xlabel('Frequency [Hz]')
	# ax_phase.set_ylabel('Phase [rad]')
	# ax_phase.set_xlim(freq[-1], freq[0])
	# ax_phase.set_ylim([-2*np.pi-.25, .25])
	# ax_phase.set_yticks([-2*np.pi, -np.pi, 0.])
	# ax_phase.grid(True, which="both")
	# ax_phase.set_yticklabels([r'$-2\pi$', r'$\pi$', '0'])


def plot_resonance_as_function_of_n_seg(n_seg_vec, resonance_freq_peak, resonance_ampl_peak, fname_snip: str = ""):
	colour = "blue"

	fig = plt.figure(figsize=(12,4))
	ax_ampl = fig.add_axes([.1, .3, .8, .6])
	ax_ampl.loglog(n_seg_vec, resonance_freq_peak, '-o', linewidth=2., color=colour)
	ax_ampl.set_xlabel('Number of segments')
	ax_ampl.set_xlabel('Resonance frequency [Hz]')
	ax_ampl.set_xlim(n_seg_vec[0], n_seg_vec[-1])
	ax_ampl.grid(True, which="both")
	fig.savefig("/tmp/freq_response_as_func_of_n_seg" + fname_snip + ".png", dpi=300)



freq = np.logspace(np.log10(float(fmin)), np.log10(float(fmax)), n_frequency_points)[::-1]		# [Hz]

n_seg_vec = [5, 10, 20, 40]

resonance_freq_peak = np.nan * np.ones_like(n_seg_vec)
resonance_ampl_peak = np.nan * np.ones_like(n_seg_vec)

for n_seg_idx, n_seg in enumerate(n_seg_vec):

	measured_ampl = np.nan * np.ones(freq.size)

	for fidx in range(len(freq)):
		f = freq[fidx]

		print("* Running simulation for n_seg = " + str(n_seg) + " and f = " + str(f) + " Hz")

		analysis_window_size = 4 / f		# [s]
		sim_time = t_analysis_start + analysis_window_size

		write_config(dt, sim_time, n_seg)

		ang_vel = generate_whisker_ang_vel(freq=f, sim_time=sim_time, ramp_up_time=ramp_up_time)
		np.savetxt("data/whisker_param_resonance/whisking_trajectory.csv", ang_vel.reshape((1, ang_vel.size)), delimiter=",")

		run_whiskit()

		ampl_base, ampl_tip = read_data('../output/resonance/kinematics/x/RC1.csv', n_segments=n_seg)

		timevec = np.linspace(0, sim_time, len(ampl_base))
		tidx_analysis_start = np.argmin((timevec - t_analysis_start)**2)
		tidx_analysis_stop = -1

		ampl_base = ang_vel[:len(ampl_tip), 2]


		#
		# 	first windowing step: select right part of timeseries
		#

		timevec_windowed = timevec[tidx_analysis_start:tidx_analysis_stop]
		x_base_windowed = ampl_base[tidx_analysis_start:tidx_analysis_stop]
		x_tip_windowed = ampl_tip[tidx_analysis_start:tidx_analysis_stop]

		n = 1 * timevec.size		# factor for zero padding
		n_analysis_points = len(timevec_windowed)
		# x_base_windowed *= np.hanning(n)ampl
		# x_tip_windowed *= np.hanning(n)theta_init

		ampl_tip_mag = np.amax(np.abs(x_tip_windowed))

		measured_ampl[fidx] = ampl_tip_mag

		# find resonance frequency (amplitude peak)
		idx_max_ampl = np.nanargmax(measured_ampl)
		resonance_freq_peak[n_seg_idx] = freq[idx_max_ampl]
		resonance_ampl_peak[n_seg_idx] = measured_ampl[idx_max_ampl]

		plot_timeseries(timevec, tidx_analysis_start, tidx_analysis_stop, ampl_base, ampl_tip, ampl_tip_mag, "/tmp/test_[f=" + str(f) + " Hz]_[n_seg=" + str(n_seg) + "].png")
		plot_resonance_curve(freq, ampl_base, measured_ampl, freq_peak=resonance_freq_peak[n_seg_idx], ampl_peak=resonance_ampl_peak[n_seg_idx], fname_snip="_[n_seg=" + str(n_seg) + "]")

	plot_resonance_as_function_of_n_seg(n_seg_vec, resonance_freq_peak, resonance_ampl_peak)
