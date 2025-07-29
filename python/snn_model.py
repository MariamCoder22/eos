from brian2 import *

# Setup basic neuron group
start_scope()
G = NeuronGroup(10, 'dv/dt = (1.0 - v) / (10*ms) : 1', threshold='v > 0.9', reset='v = 0', method='exact')
M = StateMonitor(G, 'v', record=True)

run(100*ms)

plot(M.t/ms, M.v[0])
xlabel('Time (ms)')
ylabel('v')
show()
