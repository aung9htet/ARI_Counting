import numpy as np
import pytest
from ari_counting_tools.validation import trajectory_samples, past_indices, SimDeadline

LIMITS=[{'name':f'joint{i}','lower':-1,'upper':1,'velocity':2} for i in range(5)]


def test_valid_trajectory():
    samples=trajectory_samples([0]*5,[{'positions':[0.1]*5,'time':2}],LIMITS)
    assert samples[0]==[0]*5
    assert samples[-1]==pytest.approx([0.1]*5)


@pytest.mark.parametrize('points',[
    [{'positions':[2]*5,'time':2}],
    [{'positions':[0.5]*5,'time':0.1}],
    [{'positions':[0]*4,'time':2}],
    [{'positions':[float('nan')]*5,'time':2}],
    [{'positions':[0]*5,'time':0}],
    [{'positions':[0]*5,'time':2},{'positions':[0]*5,'time':1}],
    [],
])
def test_reject_invalid(points):
    with pytest.raises(ValueError): trajectory_samples([0]*5,points,LIMITS)


def test_causal_alignment_and_staleness():
    indices,valid=past_indices([100,200,400],[50,100,150,200,350,400,550],max_age_ns=100)
    assert indices.tolist()==[-1,0,0,1,-1,2,-1]
    assert valid.tolist()==[False,True,True,True,False,True,False]
    assert past_indices([], [1,2])[0].tolist()==[-1,-1]
    with pytest.raises(ValueError): past_indices([200,100],[200])


def test_sim_deadline_counts_simulation_time():
    wall=[0.0]; deadline=SimDeadline(10,stall=30,wall=lambda:wall[0])
    assert not deadline.expired(0)                       # no /clock message yet
    wall[0]=5; assert not deadline.expired(500_000_000_000)   # budget starts at first valid time
    wall[0]=29; assert not deadline.expired(509_000_000_000)  # 24 wall s but only 9 sim s
    assert deadline.expired(510_000_000_001)


def test_sim_deadline_detects_stalled_clock():
    wall=[0.0]; deadline=SimDeadline(10,stall=30,wall=lambda:wall[0])
    assert not deadline.expired(1_000_000_000)
    wall[0]=30; assert not deadline.expired(1_000_000_000)
    wall[0]=30.1; assert deadline.expired(1_000_000_000)
