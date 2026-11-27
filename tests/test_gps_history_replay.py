from unittest.mock import Mock

import numpy as np
import pytest
from pypilot import gps_filter, sensors


class Client:
    def register(self, value):
        return value


def test_delay_follows_the_source_of_each_accepted_fix(monkeypatch):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    monkeypatch.setattr(gps_filter, 'pygeomag', None)
    client = Client()
    f = gps_filter.GPSFilter(client)
    f.enabled.set(True)
    process = gps_filter.GPSFilterProcess.__new__(gps_filter.GPSFilterProcess)
    process.pipe = Mock()
    monkeypatch.setattr(sensors, 'GPSFilterProcess', lambda client: process)
    gps = sensors.gps(client)

    assert f.gps_time_offset.value == 0.12
    for source, delay in [('signalk', 0.12), ('tcp', 0.12), ('serial', 0.09), ('gpsd', 0.09)]:
        fix = {'device': source, 'timestamp': 100., 'lat': 50., 'lon': -1., 'speed': 0.}
        assert gps.write(fix, source)
        cmd, args = process.pipe.send.call_args.args[0]
        assert cmd == 'update' and args[2] == source
        f.update(*args)
        assert f.gps_time_offset.value == delay
        assert f.gps_system_time_offset == fix['timestamp'] - args[1]

    process.pipe.send.reset_mock()
    assert not gps.write(dict(fix, device='tcp'), 'tcp')
    process.pipe.send.assert_not_called()
    gps.source.set('none')  # previous source timed out
    assert gps.write(dict(fix, device='tcp'), 'tcp')
    f.update(*process.pipe.send.call_args.args[0][1])
    assert f.gps_time_offset.value == 0.12


@pytest.fixture
def make_filter(monkeypatch):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    monkeypatch.setattr(gps_filter, 'pygeomag', None)
    def make():
        f = gps_filter.GPSFilter(Client())
        f.enabled.set(True)
        f.gps_time_offset.set(0)
        return f
    return make


def gps_fix(t, east=0., north=0., **fields):
    lat, lon = gps_filter.xy_to_ll(east, north, 50., 179.9999)
    return dict(timestamp=t, lat=lat, lon=lon, **fields)


def imu(f, t, east=0., north=0., up=0.):
    f.predict([-north/9.81, -east/9.81, 1 + up/9.81], [1, 0, 0, 0], t)


@pytest.mark.parametrize('lag', [0., .09, .12, .7])
def test_delayed_fixes_match_chronological_fusion(make_filter, lag):
    # GPS-first and IMU-first at equal times, fractional intervals, fixes
    # closer together than the delay, longitude wrap, and position-only fixes.
    fixes = [gps_fix(10), gps_fix(10.1, 11, 3, speed=3, track=25),
             gps_fix(10.104, 14, 1), gps_fix(10.239, 17, 2),
             gps_fix(10.625, 18, 3, speed=2, track=110)]
    def run(delay, imu_first):
        f = make_filter()
        events = [(10 + i/32, imu_first, 'imu', i) for i in range(51)]
        events += [(fix['timestamp'] + delay, not imu_first, 'gps', fix) for fix in fixes]
        for t, _, event, arg in sorted(events):
            if event == 'gps':
                f.update(arg, t)
            else:
                imu(f, t, east=(arg % 7) / 5, north=(arg % 3) - 1)
        return f
    reference = run(0., True)
    actual = run(lag, False)
    assert actual.lastll == reference.lastll == (fixes[-1]['lat'], fixes[-1]['lon'])
    np.testing.assert_allclose(actual.X, reference.X, atol=1e-10)
    np.testing.assert_allclose(actual.P, reference.P, atol=1e-10)
    assert actual.fix.value['lon'] == pytest.approx(reference.fix.value['lon'], abs=1e-10)


@pytest.mark.parametrize('lag', [0., .12])
def test_fractional_fix_matches_hand_calculated_position_velocity_and_covariance(make_filter, lag):
    f = make_filter()
    f.update(gps_fix(10.), 10.)
    imu(f, 10., east=2)
    # 0.01 s is shorter than the former minimum prediction interval.
    fix = gps_fix(10.01, east=3.)
    if lag == 0:
        f.update(fix, 10.01)
    imu(f, 10.1, east=-1)
    if lag:
        f.update(fix, 10.01 + lag)
    imu(f, 10.2)

    I = np.eye(2)
    def transition(dt):
        return np.block([[I, dt*I], [np.zeros((2, 2)), I]])
    H = np.eye(4)[:2]
    def noise(dt):
        return gps_filter.VELOCITY_RANDOM_WALK**2 * np.kron(
            [[dt**3/3, dt**2/2], [dt**2/2, dt]], I)
    P = np.diag([100., 100., 1., 1.])
    P = transition(.01) @ P @ transition(.01).T + noise(.01)
    X = np.array([.0001 - 3, 0., .02, 0.])
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + 100*I)
    X -= K @ (H @ X)
    P = (np.eye(4) - K @ H) @ P
    for dt, a in [(.09, 2), (.1, -1)]:
        F = transition(dt)
        X = F @ X + [dt**2*a/2, 0., dt*a, 0.]
        P = F @ P @ F.T + noise(dt)
    np.testing.assert_allclose(f.X, X, atol=1e-8)
    np.testing.assert_allclose(f.P, P, atol=1e-10)


def test_gps_outage_bounds_history_and_preserves_dead_reckoning(make_filter):
    f = make_filter()
    f.update(gps_fix(10., speed=1/gps_filter.MPS_PER_KNOT, track=0), 10.)
    for i in range(501):
        imu(f, 10 + i/32)
    assert len(f.history) <= 32*gps_filter.GPS_HISTORY_SECONDS + 1
    np.testing.assert_allclose(f.X, [0., 500/32, 0., 1.], atol=1e-9)
    X, P = f.X.copy(), f.P.copy()
    f.update(gps_fix(19., east=80), 25.625)  # before retained history
    np.testing.assert_array_equal(f.X, X)
    np.testing.assert_array_equal(f.P, P)
    f.update(gps_fix(25.25, north=15.25), 25.625)
    assert f.lastll == pytest.approx(gps_filter.xy_to_ll(0, 15.25, 50., 179.9999))
    np.testing.assert_allclose(f.X, [0., .375, 0., 1.], atol=1e-8)


def test_older_fix_cannot_undo_the_latest_correction(make_filter):
    f = make_filter()
    f.update(gps_fix(10.), 10.)
    for i in range(8):
        imu(f, 10 + i/32)
    f.update(gps_fix(10.1, east=10.), 10.2)
    X, P, origin = f.X.copy(), f.P.copy(), f.lastll
    f.update(gps_fix(10.07, east=50.), 10.23)
    np.testing.assert_array_equal(f.X, X)
    np.testing.assert_array_equal(f.P, P)
    assert f.lastll == origin


def test_same_fix_can_gain_velocity_without_counting_position_twice(make_filter):
    partial, complete = make_filter(), make_filter()
    position = gps_fix(10.08, 2., 1.)
    with_velocity = dict(position, track=45, speed=2.)
    for f in (partial, complete):
        f.update(gps_fix(10.), 10.)
        for i in range(6):
            imu(f, 10+i/32, east=.2, north=.4)
    partial.update(position, 10.16)
    partial.update(with_velocity, 10.17)
    complete.update(with_velocity, 10.17)
    complete.update(with_velocity, 10.18)  # retransmission of the same fix
    np.testing.assert_allclose(partial.X, complete.X, atol=1e-9)
    np.testing.assert_allclose(partial.P, complete.P, atol=1e-9)


def test_delayed_first_fix_supplies_declination_for_buffered_imu(make_filter, monkeypatch):
    model = Mock()
    model.GeoMag.return_value.calculate.return_value.d = 90.
    early, late = make_filter(), make_filter()
    monkeypatch.setattr(gps_filter, 'pygeomag', model)
    for f in (early, late):
        imu(f, 10., north=1.)
        if f is early:
            f.update(gps_fix(10.01), 10.01)
        imu(f, 10.1, north=1.)
        imu(f, 10.2, north=1.)
        if f is late:
            f.update(gps_fix(10.01), 10.21)
    np.testing.assert_allclose(late.X, [.5*.19**2, 0., .19, 0.], atol=1e-10)
    np.testing.assert_allclose(early.X, late.X)
    np.testing.assert_allclose(early.P, late.P)


def test_earlier_gps_delivery_keeps_inertial_velocity(make_filter):
    f = make_filter()
    utc = 1_800_000_000.
    f.update(gps_fix(utc, speed=1/gps_filter.MPS_PER_KNOT, track=90.), 10.14, 'serial')
    for i in range(21):
        imu(f, 10.15 + i/20)
    # This position-only fix took 50 ms less time to arrive than the first.
    f.update(gps_fix(utc + 1, east=.95), 11.09, 'serial')
    np.testing.assert_allclose(f.X[2:], [1., 0.], atol=1e-5)
    assert f.history and f.checkpoint[0] == pytest.approx(11., abs=1e-6)


def test_long_gap_between_imu_samples_restarts_history(make_filter):
    f = make_filter()
    imu(f, 10.)
    f.update(gps_fix(10.01), 10.01)
    imu(f, 10.1, north=1.)
    imu(f, 12.)
    assert f.checkpoint is None
    f.update(gps_fix(12.01, speed=1/gps_filter.MPS_PER_KNOT, track=90.), 12.11)
    imu(f, 12.13)
    np.testing.assert_allclose(f.X, [.12, 0., 1., 0.], atol=1e-10)


def test_fresh_gps_fix_can_arrive_before_imu_resumes(make_filter):
    f = make_filter()
    f.update(gps_fix(10.), 10.)
    imu(f, 10.1, east=2.)
    f.update(gps_fix(13., north=5, speed=1/gps_filter.MPS_PER_KNOT, track=0.), 13.1)
    imu(f, 13.15)
    np.testing.assert_allclose(f.X, [0., .15, 0., 1.], atol=1e-10)


def test_delayed_altitude_and_climb_in_3d(make_filter):
    states = []
    for lag in (0., .12):
        f = make_filter()
        f.use3d = True
        f.reset()
        f.update(gps_fix(10., alt=100., climb=1.), 10.)
        imu(f, 10., up=2.)
        fix = gps_fix(10.05, alt=100.4)
        if not lag:
            f.update(fix, 10.05)
        imu(f, 10.125, up=-1.)
        if lag:
            f.update(fix, 10.05+lag)
        imu(f, 10.25)
        assert f.X[2] > 100.25 and f.X[5] > 1.
        states.append((f.X, f.P))
    for early, late in zip(*states):
        np.testing.assert_allclose(early, late)


def test_low_weight_delayed_gps_fix_does_not_move_boat_to_new_origin(make_filter):
    f = make_filter()
    f.update(gps_fix(10.), 10.)
    for i in range(21):
        imu(f, 10+i/20)
    f.position_error.set(1000)
    f.update(gps_fix(10.3, east=4.), 11.)
    lat, lon = gps_filter.xy_to_ll(*f.X[:2], *f.lastll)
    east, north = gps_filter.ll_to_xy(lat, lon, 50., 179.9999)
    assert abs(east) < .001 and abs(north) < .001


@pytest.mark.parametrize('dt_list', [
    [.1]*10, [.05]*20, [.025]*40, [.01, .23, .42, .34]
])
def test_one_second_covariance_independent_of_sample_rate(make_filter, dt_list):
    f = make_filter()
    f.update(gps_fix(10., speed=0., track=0.), 10.)
    imu(f, 10.)
    t = 10.
    for dt in dt_list:
        t += dt
        imu(f, t)
    # Analytical covariance after one second, including uncertainty in the first fix.
    vel_var = (.5 * 1852/3600)**2
    q = gps_filter.VELOCITY_RANDOM_WALK**2
    expected = [[100 + vel_var + q/3, vel_var + q/2], [vel_var + q/2, vel_var + q]]
    np.testing.assert_allclose(f.P, np.kron(expected, np.eye(2)), atol=1e-11)


def test_gps_standard_deviations_are_live_persistent_settings(make_filter):
    f = make_filter()
    for setting, units, value in [(f.position_error, 'meters', 10), (f.velocity_error, 'knots', .5)]:
        assert setting.info['persistent'] and setting.info['writable']
        assert setting.info['units'] == units and setting.value == value
    f.use3d = True
    f.reset()
    fix = dict(speed=0., track=0., alt=10., climb=0.)
    f.update(gps_fix(10., **fix), 10.)
    mps = .5 * 1852/3600
    initial_variances = np.square([10, 10, 20, mps, mps, 2*mps])
    np.testing.assert_allclose(f.P, np.diag(initial_variances))
    # Halving both sigmas quarters the variances of the next observation.
    f.position_error.set(5.)
    f.velocity_error.set(.25)
    dt = .1
    _, prior = f.predict_interval(f.X, f.P, dt, None)
    f.update(gps_fix(10. + dt, **fix), 10. + dt)
    expected = np.linalg.inv(np.linalg.inv(prior) + np.diag(4 / initial_variances))
    np.testing.assert_allclose(f.P, expected, atol=1e-11)


def test_first_fix_split_across_messages_has_same_uncertainty(make_filter):
    partial, complete = make_filter(), make_filter()
    position = gps_fix(10.)
    full = dict(position, speed=5., track=45.)
    partial.update(position, 10.)
    imu(partial, 10.05, east=.2)
    partial.update(full, 10.1)
    complete.update(full, 10.)
    imu(complete, 10.05, east=.2)
    np.testing.assert_allclose(partial.X, complete.X)
    np.testing.assert_allclose(partial.P, complete.P)
