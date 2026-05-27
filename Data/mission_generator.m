t = [0, 5, 10, 20, 30, 30.001, 41, 46, 46.001, 65, 70, 80];
h = [0, 0, 1.0, 0.2, 0.2, 1.1, 1.1, 0.25, 0.9, 0.5, 0, 0];

ref_signal = timeseries(h, t);
ref_signal.Name = 'Reference_Height';