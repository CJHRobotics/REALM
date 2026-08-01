import math
import numpy as np


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pose_slice(t):
    """Index range in the (horizontal) stacked state vector for pose x_t."""
    return slice(3 * t, 3 * t + 3)


def landmark_slice(j, n_poses):
    """Index range in the stacked state vector for landmark m_j."""
    start = 3 * n_poses + 2 * j
    return slice(start, start + 2)


def graphSLAM_init(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0)):
    """
    Builds the initial linearization point: dead-reckons pose estimates
    from odometry_log, then seeds each landmark's (x, y) estimate from its
    first non-NaN observation.

    odometry_log[0] is the null edge captured before the loop started
    (near-zero motion into x0) and is skipped as a real motion edge.

    Parameters
    ----------
    odometry_log : list of (v, omega, dt)
    landmark_log  : list of (n_landmarks, 2) arrays of (range, bearing),
                    NaN where a landmark wasn't visible that timestep
    x0            : initial pose, (x, y, theta)

    Returns
    -------
    mu_poses     : (T, 3) ndarray of [x, y, theta] pose estimates
    mu_landmarks : (n_landmarks, 2) ndarray of [x, y] landmark estimates,
                   NaN rows for landmarks never observed
    """

    timesteps = len(odometry_log)
    #creates an array corresponding to the poses at each timestep
    mu_poses = np.zeros((timesteps, 3))
    #index first pose as (x=0,y=0,theta=0)
    mu_poses[0] = x0

    #given the linear and angular velocity at time t
    #compute the belief pose
    for t in range(1, timesteps):
        v, omega, dt = odometry_log[t]
        x, y, theta = mu_poses[t - 1]

        #poses are estimated from velocity times time (distance)
        #then multiplied by cos(theta) sin and (theta)
        #as these are decomposed into components
        #omega *dt gives angular displacement which is added to theta and normalized
        mu_poses[t] = [
            x + v * dt * math.cos(theta),
            y + v * dt * math.sin(theta),
            normalize_angle(theta + omega * dt),
        ]


    n_landmarks = landmark_log[0].shape[0]
    #mu_landmarks stores position of landmark when it is first seen
    #the position is calculated by turning the relative position into a
    #global position
    mu_landmarks = np.full((n_landmarks, 2), np.nan, dtype=np.float64)
    for t, row in enumerate(landmark_log):
        for j in range(n_landmarks):
            if np.isnan(mu_landmarks[j, 0]) and not np.isnan(row[j, 0]):
                r, phi = row[j]
                x, y, theta = mu_poses[t]
                bearing = theta + phi
                mu_landmarks[j] = [
                    x + r * math.cos(bearing),
                    y + r * math.sin(bearing),
                ]

    return mu_poses, mu_landmarks


def graphSLAM_linearize(mu_poses, mu_landmarks, odometry_log, landmark_log,
                         R=None, Q=None, anchor_information=1e9):
    """
    One Gauss-Newton linearization pass. Builds the information matrix
    Omega and information vector xi such that solving

        Omega @ delta = xi

    gives the CORRECTION delta to apply on top of the current estimate:

        mu_poses_new     = mu_poses     + delta[:3*T].reshape(T, 3)
        mu_landmarks_new = mu_landmarks + delta[3*T:].reshape(M, 2)

    State vector layout: [x_0, x_1, ..., x_{T-1}, m_0, m_1, ..., m_{M-1}],
    each x_t = (x, y, theta), each m_j = (x, y).

    Parameters
    ----------
    mu_poses     : (T, 3) current pose estimates
    mu_landmarks : (M, 2) current landmark estimates (NaN row = unobserved)
    odometry_log : list of (v, ang_v, dt), same length as mu_poses
    landmark_log : list of (M, 2) arrays of (range, bearing), same length
                   as mu_poses
    R            : (3, 3) motion noise covariance
    Q            : (2, 2) measurement noise covariance
    anchor_information : information (not covariance) pinning x_0 in place

    Returns
    -------
    Omega : (3T + 2M, 3T + 2M) ndarray
    xi    : (3T + 2M) ndarray
    """
    num_timesteps = len(mu_poses)
    num_landmarks = len(mu_landmarks)
    total_dimensions = 3 * num_timesteps + 2 * num_landmarks

    #noise estimates
    if R is None:
        R = np.diag([0.05 ** 2, 0.05 ** 2, math.radians(2) ** 2])
    if Q is None:
        Q = np.diag([0.05 ** 2, math.radians(3) ** 2])

    R_inv = np.linalg.inv(R)
    Q_inv = np.linalg.inv(Q)

    #information matrix and vector
    Omega = np.zeros((total_dimensions, total_dimensions))
    xi = np.zeros(total_dimensions)

    # Anchor x_0: heavily penalize moving it, so delta_x0 stays ~0 and the
    # whole trajectory/map is pinned to an absolute frame.
    x0_idx = pose_slice(0) #returns slice object of pose at time 0
    Omega[x0_idx, x0_idx] += anchor_information * np.eye(3)


    # Motion edges: x_{t-1} -> x_t via odometry_log[t]
    for t in range(1, num_timesteps):
        v, ang_v, dt = odometry_log[t]
        x, y, theta = mu_poses[t - 1]

        predicted_pose = np.array([
            x + v * dt * math.cos(theta),
            y + v * dt * math.sin(theta),
            normalize_angle(theta + ang_v * dt),
        ])

        motion_jacobian = np.array([
            [1, 0, -v * dt * math.sin(theta)],
            [0, 1, v * dt * math.cos(theta)],
            [0, 0, 1],
        ])

        # residual = mu_poses[t] - predicted_pose;
        # d(residual)/d[prev_pose, curr_pose] = [-motion_jacobian, I]

        residual_jacobian = np.hstack([-motion_jacobian, np.eye(3)])

        residual = mu_poses[t] - predicted_pose
        residual[2] = normalize_angle(residual[2])


        idx = np.r_[np.arange(3 * (t - 1), 3 * (t - 1) + 3),
                    np.arange(3 * t, 3 * t + 3)]

        Omega[np.ix_(idx, idx)] += residual_jacobian.T @ R_inv @ residual_jacobian
        xi[idx] += -residual_jacobian.T @ R_inv @ residual

    # Measurement edges: x_t -> m_j via landmark_log[t][j]
    for t, row in enumerate(landmark_log):
        x, y, theta = mu_poses[t]
        for j in range(num_landmarks):
            r_obs, phi_obs = row[j]
            if np.isnan(r_obs) or np.isnan(mu_landmarks[j, 0]):
                continue

            mx, my = mu_landmarks[j]
            dx, dy = mx - x, my - y
            q = dx ** 2 + dy ** 2
            r_hat = math.sqrt(q)
            phi_hat = normalize_angle(math.atan2(dy, dx) - theta)

            # H = d(h)/d[x, y, theta, mx, my], h = [r_hat, phi_hat]
            H = np.array([
                [-dx / r_hat, -dy / r_hat, 0, dx / r_hat, dy / r_hat],
                [dy / q, -dx / q, -1, -dy / q, dx / q],
            ])

            # residual r = z_obs - h(x); d(r)/d[...] = -H
            residual = np.array([
                r_obs - r_hat,
                normalize_angle(phi_obs - phi_hat),
            ])

            pose_idx = np.arange(3 * t, 3 * t + 3)
            lm_idx = np.arange(3 * num_timesteps + 2 * j, 3 * num_timesteps + 2 * j + 2)
            idx = np.r_[pose_idx, lm_idx]

            Omega[np.ix_(idx, idx)] += H.T @ Q_inv @ H
            xi[idx] += H.T @ Q_inv @ residual

    return Omega, xi


def graphSLAM_reduce(Omega, xi, num_timesteps, num_landmarks):
    """
    Eliminates the landmark variables from the full system via the Schur
    complement, leaving a pose-only system:

        Omega_reduced = Omega_xx - sum_j Omega_xm_j @ inv(Omega_mm_j) @ Omega_mx_j
        xi_reduced    = xi_x     - sum_j Omega_xm_j @ inv(Omega_mm_j) @ xi_m_j

    This works landmark-by-landmark (rather than inverting one big
    Omega_mm block) because landmarks never connect to each other in the
    graph -- only to the poses that observed them -- so Omega_mm is
    block-diagonal and each landmark's 2x2 block can be eliminated on its
    own.

    Landmarks that were never observed have an all-zero Omega_mm block
    (no measurement edges ever touched them in linearize) and are skipped,
    since that block can't be inverted and carries no information anyway.

    Parameters
    ----------
    Omega : (3T + 2M, 3T + 2M) ndarray, from graphSLAM_linearize
    xi    : (3T + 2M,) ndarray, from graphSLAM_linearize
    num_timesteps : T
    num_landmarks : M

    Returns
    -------
    Omega_reduced : (3T, 3T) ndarray, pose-only information matrix
    xi_reduced    : (3T,) ndarray, pose-only information vector
    """
    pose_dim = 3 * num_timesteps

    Omega_reduced = Omega[:pose_dim, :pose_dim].copy()
    xi_reduced = xi[:pose_dim].copy()

    for j in range(num_landmarks):
        lm_idx = landmark_slice(j, num_timesteps)

        omega_mm_j = Omega[lm_idx, lm_idx]
        if np.allclose(omega_mm_j, 0):
            #never observed -- no information to eliminate
            continue

        omega_mm_j_inv = np.linalg.inv(omega_mm_j)

        omega_xm_j = Omega[:pose_dim, lm_idx]  #(3T, 2), sparse: only nonzero
                                                #for poses that saw landmark j
        xi_m_j = xi[lm_idx]                    #(2,)

        Omega_reduced -= omega_xm_j @ omega_mm_j_inv @ omega_xm_j.T
        xi_reduced -= omega_xm_j @ omega_mm_j_inv @ xi_m_j

    return Omega_reduced, xi_reduced


def graphSLAM_solve(Omega, xi, Omega_reduced, xi_reduced, num_timesteps, num_landmarks):
    """
    Solves the reduced pose-only system for the pose correction, then
    back-substitutes each landmark's own block from the FULL (unreduced)
    Omega/xi -- together with the now-known pose correction -- to recover
    that landmark's correction:

        delta_m_j = inv(Omega_mm_j) @ (xi_m_j - Omega_mx_j @ delta_poses)

    Landmarks that were never observed (skipped during reduce, since their
    Omega_mm block can't be inverted) get a correction of exactly 0 --
    there's no evidence to move them.

    Parameters
    ----------
    Omega, xi             : full system from graphSLAM_linearize
    Omega_reduced, xi_reduced : pose-only system from graphSLAM_reduce
    num_timesteps : T
    num_landmarks : M

    Returns
    -------
    delta_poses     : (T, 3) ndarray, pose corrections
    delta_landmarks : (M, 2) ndarray, landmark corrections
    """
    pose_dim = 3 * num_timesteps

    delta_poses_flat = np.linalg.solve(Omega_reduced, xi_reduced)
    delta_poses = delta_poses_flat.reshape(num_timesteps, 3)

    delta_landmarks = np.zeros((num_landmarks, 2))
    for j in range(num_landmarks):
        lm_idx = landmark_slice(j, num_timesteps)

        omega_mm_j = Omega[lm_idx, lm_idx]
        if np.allclose(omega_mm_j, 0):
            #never observed -- leave its correction at 0
            continue

        omega_mm_j_inv = np.linalg.inv(omega_mm_j)
        omega_mx_j = Omega[lm_idx, :pose_dim]  #(2, 3T)
        xi_m_j = xi[lm_idx]                    #(2,)

        delta_landmarks[j] = omega_mm_j_inv @ (xi_m_j - omega_mx_j @ delta_poses_flat)

    return delta_poses, delta_landmarks


def graphSLAM_run(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0), R=None, Q=None,
                   anchor_information=1e9, max_iterations=20, tolerance=1e-4,
                   verbose=False):
    """
    Runs full offline GraphSLAM: init once, then repeatedly
    linearize -> reduce -> solve -> apply the correction, until the
    correction gets small (converged) or max_iterations is hit.

    Repeating is necessary because the motion/measurement models are
    nonlinear -- one linearize/reduce/solve pass is only a Gauss-Newton
    step around the CURRENT estimate, not an exact answer. Each pass
    re-linearizes around the updated mu_poses/mu_landmarks from the
    previous pass.

    Parameters
    ----------
    odometry_log, landmark_log : see graphSLAM_init / graphSLAM_linearize
    x0                  : initial pose, (x, y, theta)
    R, Q                : motion / measurement noise covariances
    anchor_information  : information pinning x_0 in place
    max_iterations      : stop after this many passes even if not converged
    tolerance           : stop early once the largest correction (pose or
                           landmark, mixing meters and radians) drops
                           below this
    verbose             : print the largest correction each iteration

    Returns
    -------
    mu_poses     : (T, 3) final pose estimates
    mu_landmarks : (M, 2) final landmark estimates
    """
    mu_poses, mu_landmarks = graphSLAM_init(odometry_log, landmark_log, x0=x0)
    num_timesteps = len(mu_poses)
    num_landmarks = len(mu_landmarks)

    for iteration in range(max_iterations):
        Omega, xi = graphSLAM_linearize(
            mu_poses, mu_landmarks, odometry_log, landmark_log,
            R=R, Q=Q, anchor_information=anchor_information,
        )
        Omega_reduced, xi_reduced = graphSLAM_reduce(Omega, xi, num_timesteps, num_landmarks)
        delta_poses, delta_landmarks = graphSLAM_solve(
            Omega, xi, Omega_reduced, xi_reduced, num_timesteps, num_landmarks,
        )

        mu_poses = mu_poses + delta_poses
        mu_poses[:, 2] = normalize_angle(mu_poses[:, 2])
        #NaN (never-observed) landmarks get a 0 delta, so NaN + 0 = NaN --
        #they stay unestimated rather than silently becoming (0, 0)
        mu_landmarks = mu_landmarks + delta_landmarks

        max_delta = max(np.max(np.abs(delta_poses)), np.nanmax(np.abs(delta_landmarks)))
        if verbose:
            print(f"iteration {iteration}: max correction = {max_delta}")

        if max_delta < tolerance:
            break

    return mu_poses, mu_landmarks

#diagnostics below ---------------------------------------------------------------
def graphSLAM_accuracy(mu_poses, dead_reckoned_poses, truth_log):
    """
    Compares GraphSLAM's optimized trajectory against dead reckoning alone,
    both measured against ground truth, to answer "did the optimization
    actually help, and by how much".

    truth_log entries are (x, y, theta_degrees) -- theta comes from
    get_compass_reading(), which is DEGREES, while mu_poses/
    dead_reckoned_poses are in RADIANS -- so headings are converted before
    comparing.

    Parameters
    ----------
    mu_poses             : (T, 3) ndarray, graphSLAM_run's final poses
    dead_reckoned_poses  : (T, 3) ndarray, graphSLAM_init's raw poses
                            (the "before" baseline, no landmark correction)
    truth_log            : list of (x, y, theta_degrees), same length as
                            mu_poses

    Returns
    -------
    dict with mean/rmse/max position error (meters) and mean heading error
    (degrees) for both mu_poses and dead_reckoned_poses, plus
    improvement_pct: how much smaller graphSLAM's mean position error is
    than dead reckoning's, as a percentage (100% = perfect, i.e. zero
    error; 0% = no better than dead reckoning; negative = worse).
    """
    truth = np.array(truth_log, dtype=np.float64)
    truth_theta_rad = normalize_angle(np.radians(truth[:, 2]))

    def position_error(poses):
        return np.hypot(poses[:, 0] - truth[:, 0], poses[:, 1] - truth[:, 1])

    def heading_error_deg(poses):
        diff = normalize_angle(poses[:, 2] - truth_theta_rad)
        return np.degrees(np.abs(diff))

    slam_pos_err = position_error(mu_poses)
    dr_pos_err = position_error(dead_reckoned_poses)

    slam_mean = slam_pos_err.mean()
    dr_mean = dr_pos_err.mean()
    #100% = graphSLAM eliminated all position error relative to dead
    #reckoning's error; 0% = no improvement; negative = graphSLAM made it worse
    improvement_pct = 100.0 * (1.0 - slam_mean / dr_mean) if dr_mean > 0 else 0.0

    return {
        "slam_mean_position_error_m": slam_mean,
        "slam_rmse_position_error_m": math.sqrt(np.mean(slam_pos_err ** 2)),
        "slam_max_position_error_m": slam_pos_err.max(),
        "slam_mean_heading_error_deg": heading_error_deg(mu_poses).mean(),
        "dead_reckoning_mean_position_error_m": dr_mean,
        "dead_reckoning_max_position_error_m": dr_pos_err.max(),
        "improvement_over_dead_reckoning_pct": improvement_pct,
    }


def print_graphSLAM_accuracy(mu_poses, dead_reckoned_poses, truth_log):
    """Prints graphSLAM_accuracy's report in a readable form."""
    report = graphSLAM_accuracy(mu_poses, dead_reckoned_poses, truth_log)
    print("GraphSLAM accuracy vs. ground truth")
    print(f"  dead reckoning mean position error: {report['dead_reckoning_mean_position_error_m']:.3f} m "
          f"(max {report['dead_reckoning_max_position_error_m']:.3f} m)")
    print(f"  graphSLAM      mean position error: {report['slam_mean_position_error_m']:.3f} m "
          f"(rmse {report['slam_rmse_position_error_m']:.3f} m, max {report['slam_max_position_error_m']:.3f} m)")
    print(f"  graphSLAM      mean heading error:  {report['slam_mean_heading_error_deg']:.2f} deg")
    print(f"  improvement over dead reckoning:    {report['improvement_over_dead_reckoning_pct']:.1f}%")
    return report


def print_landmark_triangulation_check(mu_landmarks, true_landmark_positions, label=None):
    """
    Compares landmark position estimates against the known true landmark
    positions from the environment. Works with EITHER graphSLAM_init's raw
    single-sighting estimates OR graphSLAM_run's fully optimized estimates
    -- pass whichever mu_landmarks you want checked, and use `label` to say
    which one it is so the printed output isn't ambiguous about which
    stage produced it.

    Comparing the init-only version isolates get_landmark_row's egocentric
    bearing + graphSLAM_init's theta + phi world-frame triangulation from
    everything else in the pipeline (linearize/reduce/solve, R/Q
    weighting). If those are already far off, the bug is in the bearing
    convention or correspondence, not the optimization -- tightening Q
    would only make things worse by trusting bad data harder.

    Parameters
    ----------
    mu_landmarks : (M, 2) ndarray, NaN row = unobserved. From EITHER
                   graphSLAM_init or graphSLAM_run.
    true_landmark_positions : (M, 2) array-like of true (x, y), same
                               indexing as mu_landmarks (e.g.
                               [(lm.x, lm.y) for lm in robot.maze.landmarks])
    label : str, optional -- describes which mu_landmarks this is (e.g.
            "graphSLAM_init (no optimization)" or "graphSLAM_run (optimized)").
            Printed as-is so the output is unambiguous about its source.
    """
    true_pos = np.array(true_landmark_positions, dtype=np.float64)
    header = "Landmark triangulation check"
    if label:
        header += f" -- {label}"
    print(header)
    for j in range(len(mu_landmarks)):
        if np.isnan(mu_landmarks[j, 0]):
            print(f"  landmark {j}: never observed")
            continue
        err = np.hypot(mu_landmarks[j, 0] - true_pos[j, 0], mu_landmarks[j, 1] - true_pos[j, 1])
        print(f"  landmark {j}: estimated ({mu_landmarks[j,0]:.2f}, {mu_landmarks[j,1]:.2f}) "
              f"true ({true_pos[j,0]:.2f}, {true_pos[j,1]:.2f})  error {err:.2f} m")


def print_landmark_observation_spread(mu_poses, landmark_log):
    """
    For each landmark, reports how many times it was observed and how
    wide a range of viewing angles it was observed from (world-frame
    bearing from the robot's current pose estimate to the landmark).

    A landmark seen only a few times from a narrow angular window has
    poor triangulation geometry -- there's little parallax to pin down
    its position confidently -- no matter how tightly Q trusts each
    individual sighting. This is meant to be read alongside
    print_landmark_triangulation_check: landmarks with large triangulation
    error AND a narrow/sparse observation spread here are explained by
    poor geometry, not a bug.
    """
    n_landmarks = landmark_log[0].shape[0]
    print("Landmark observation spread")
    for j in range(n_landmarks):
        bearings = []
        ranges = []
        for t, row in enumerate(landmark_log):
            r_obs, phi_obs = row[j]
            if np.isnan(r_obs):
                continue
            x, y, theta = mu_poses[t]
            bearings.append(normalize_angle(theta + phi_obs))
            ranges.append(r_obs)

        if not bearings:
            print(f"  landmark {j}: never observed")
            continue

        bearings = np.array(bearings)
        #circular mean, so spread doesn't get corrupted by the -180/180 wrap
        mean_bearing = math.atan2(np.mean(np.sin(bearings)), np.mean(np.cos(bearings)))
        deviations = np.array([normalize_angle(b - mean_bearing) for b in bearings])
        spread_deg = math.degrees(deviations.max() - deviations.min())

        print(f"  landmark {j}: {len(bearings)} sightings, "
              f"viewing-angle spread {spread_deg:.1f} deg, "
              f"range {min(ranges):.2f}-{max(ranges):.2f} m")


def print_worst_pose_errors(mu_poses, truth_log, odometry_log, top_n=10):
    """
    Reports the top_n timesteps with the largest position error between
    mu_poses and truth_log, alongside the odometry_log entry that produced
    the transition INTO that pose (odometry_log[t], connecting x_{t-1} to
    x_t). Useful for checking whether the worst pose errors line up with
    odometry outliers (e.g. encoder-derived velocity spikes) rather than
    being spread evenly across the run -- a concentrated pattern points at
    a few untrustworthy motion edges being over-trusted by R, not a
    systematic bug in the optimization itself.
    """
    truth = np.array(truth_log, dtype=np.float64)
    pos_err = np.hypot(mu_poses[:, 0] - truth[:, 0], mu_poses[:, 1] - truth[:, 1])
    worst = np.argsort(pos_err)[::-1][:top_n]

    print(f"Worst {top_n} pose errors")
    for t in worst:
        v, omega, dt = odometry_log[t]
        print(f"  t={t}: error {pos_err[t]:.3f} m  |  "
              f"odometry_log[t] = (v={v:.3f}, omega={omega:.3f}, dt={dt:.3f})")
