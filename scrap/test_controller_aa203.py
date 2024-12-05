

def get_QR_MPC(len_state, len_u):
    # Q and R
    Q = np.identity(len_state) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(len_u)

    Q[0][0] = 1 / (Q_X ** 2)
    Q[1][1] = 1 / (Q_Y ** 2)
    Q[2][2] = 1 / (Q_Z ** 2)
    Q[3][3] = 1 / (Q_VX ** 2)
    Q[4][4] = 1 / (Q_VY ** 2)
    Q[5][5] = 1 / (Q_VZ ** 2)
    Q[6][6] = 1 / (Q_PIT ** 2)
    Q[7][7] = 1 / (Q_YAW ** 2)
    Q[9][9] = 1 / (Q_VPIT ** 2)
    Q[10][10] = 1 / (Q_VYAW ** 2)
    
    R[0][0] = 1 / (R_X ** 2)
    R[1][1] = 1 / (R_Y ** 2)
    R[2][2] = 1 / (R_T ** 2)
    return Q, R


def get_QR_LQR(len_state, len_u):
    # Q and R
    Q = np.identity(len_state - 2 + 3) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(len_u)

    Q[0][0] = 1 / (Q_X ** 2)
    Q[1][1] = 1 / (Q_Y ** 2)
    Q[2][2] = 1 / (Q_Z ** 2)
    Q[3][3] = 1 / (Q_VX ** 2)
    Q[4][4] = 1 / (Q_VY ** 2)
    Q[5][5] = 1 / (Q_VZ ** 2)
    Q[6][6] = 1 / (Q_PIT ** 2)
    Q[7][7] = 1 / (Q_YAW ** 2)
    Q[8][8] = 1 / (Q_VPIT ** 2)
    Q[9][9] = 1 / (Q_VYAW ** 2)
    Q[10][10] = 1 / (Q_X ** 2)
    Q[11][11] = 1 / (Q_Y ** 2)
    Q[12][12] = 1 / (Q_Z ** 2)
    
    R[0][0] = 1 / (R_X ** 2)
    R[1][1] = 1 / (R_Y ** 2)
    R[2][2] = 1 / (R_T ** 2)
    return Q, R

def get_QR_MPC(len_state, len_u):
    # Q and R
    Q = np.identity(len_state) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(len_u)

    Q[0][0] = 1 / (Q_X ** 2)
    Q[1][1] = 1 / (Q_Y ** 2)
    Q[2][2] = 1 / (Q_Z ** 2)
    Q[3][3] = 1 / (Q_VX ** 2)
    Q[4][4] = 1 / (Q_VY ** 2)
    Q[5][5] = 1 / (Q_VZ ** 2)
    Q[6][6] = 1 / (Q_PIT ** 2)
    Q[7][7] = 1 / (Q_YAW ** 2)
    Q[9][9] = 1 / (Q_VPIT ** 2)
    Q[10][10] = 1 / (Q_VYAW ** 2)
    
    R[0][0] = 1 / (R_X ** 2)
    R[1][1] = 1 / (R_Y ** 2)
    R[2][2] = 1 / (R_T ** 2)
    return Q, R


def do_MPC(A, B, t, ts, tf, current_state, linearized_u, ideal_trajectory):
    
    # Get target state at end of time horizon
    if int(t/ts) + TIMESTEP_HORIZON > int(tf/ts):
        target_state = ideal_trajectory[-1]
        steps_until_target = int(tf/ts) - int(t/ts)
    else:
        target_state = ideal_trajectory[int(t/ts) + TIMESTEP_HORIZON]
        steps_until_target = TIMESTEP_HORIZON
    
    # Solve open loop optimal control problem
    n = len(current_state)
    m = len(linearized_u)
    N = steps_until_target
    A = np.eye(12) + A*ts
    B = B*ts
    Q, R = get_QR_MPC(n, m)
    x_mpc = cvx.Variable((N+1, n))
    u_mpc = cvx.Variable((N+1, m))
    objective = 0
    constraints = [
        x_mpc[0] == current_state
    ]
    for k in range(steps_until_target):
        objective += cvx.quad_form(x_mpc[-1] - target_state, Q) + cvx.quad_form(u_mpc[-1], R)
        constraints += [
            x_mpc[k+1] == A@x_mpc[k] + B@u_mpc[k]
        ]
    
    prob = cvx.Problem(cvx.Minimize(objective), constraints)
    prob.solve()
    status = prob.status
    try:
        u = u_mpc.value[0] + linearized_u
    except:
        u = np.array([0,0,0])
    return u

from scipy.linalg import solve_continuous_are, solve, schur
from scipy.linalg import qr, ordqz, solve_triangular, lu, solve_triangular, norm, LinAlgError, matrix_balance, block_diag
from numpy.linalg import cond
from numpy.linalg import inv, LinAlgError

def our_LQR(A, B, Q, R, C):
    """
    Linear Quadratic Regulator (LQR) design with integral action.
    
    Parameters
    ----------
    A : 2D array
        Dynamics matrix.
    B : 2D array
        Input matrix.
    Q : 2D array
        State weighting matrix.
    R : 2D array
        Input weighting matrix.
    C : 2D array, optional
        Integral action matrix.
    
    Returns
    -------
    K : 2D array
        State feedback gains.
    X : 2D array
        Solution to Riccati equation.
    E : 1D array
        Eigenvalues of the closed loop system.
    """
    n_states = A.shape[0]
    n_inputs = B.shape[1]

    if C is not None:
        # Ensure C has the correct shape
        assert C.shape[1] == n_states, "C matrix must have the same number of columns as A"

        # Process the states to be integrated
        n_integrators = C.shape[0]

        # Augment the system with integrators
        A_aug = np.block([
            [A, np.zeros((n_states, n_integrators))],
            [C, np.zeros((n_integrators, n_integrators))]
        ])
        B_aug = np.vstack([B, np.zeros((n_integrators, n_inputs))])
    else:
        A_aug = A
        B_aug = B

    # Solve the Riccati equation for the augmented system
    X = solve_Riccati(A_aug, B_aug, Q, R)

    # Calculate the LQR gain
    K = np.linalg.inv(R) @ B_aug.T @ X

    return K

def solve_Riccati(A, B, Q, R):
    """
    Solves the continuous-time algebraic Riccati equation (CARE).
    
    Parameters
    ----------
    A : 2D array
        Dynamics matrix.
    B : 2D array
        Input matrix.
    Q : 2D array
        State weighting matrix.
    R : 2D array
        Input weighting matrix.
    
    Returns
    -------
    X : 2D array
        Solution to the Riccati equation.
    """
    try:
        g = inv(R)
    except LinAlgError:
        raise ValueError('Matrix R is singular')

    # Compute g = B * inv(R) * B^T
    g = np.dot(np.dot(B, g), B.conj().transpose())

    # Construct the Hamiltonian matrix Z
    z11 = A
    z12 = -1.0*g
    z21 = -1.0*Q
    z22 = -1.0*A.conj().transpose()
    
    z = np.vstack((np.hstack((z11, z12)), np.hstack((z21, z22))))
    
    # Perform the Schur decomposition of Z
    [s, u, sorted] = schur(z, sort='lhp')

    (m, n) = u.shape

    # Extract the upper-left and lower-left blocks of the unitary matrix U
    u11 = u[0:m//2, 0:n//2]
    u21 = u[m//2:m, 0:n//2]
    u11i = inv(u11)

    # Return the solution to the Riccati equation
    return np.dot(u21, u11i)

def solve_Riccati1(A, B, Q, R):  
    """
    Iteratively solves the continuous-time algebraic Riccati equation (CARE).
    
    Parameters
    ----------
    A : 2D array
        Dynamics matrix.
    B : 2D array
        Input matrix.
    Q : 2D array
        State weighting matrix.
    R : 2D array
        Input weighting matrix.
    
    Returns
    -------
    K : 2D array
        State feedback gains.
    """
    # Normalize matrices to avoid numerical issues
    Q = (Q + np.eye(Q.shape[0]) * 1e-6) / np.max(np.abs(Q))
    R = (R + np.eye(R.shape[0]) * 1e-6) / np.max(np.abs(R))
    A = A / np.max(np.abs(A))
    B = B / np.max(np.abs(B))

    max_iters = 10000
    eps = 1e-9

    n = A.shape[0]
    P_prev = np.eye(n) # Initial guess
    converged = False

    for i in range(max_iters):
        # Solve for the gain matrix K
        K = -np.linalg.solve(R + B.T @ P_prev @ B, B.T @ P_prev @ A)

        # Update the Riccati equation solution P
        P = Q + A.T @ P_prev @ (A + B @ K)
        
        # Check for convergence
        if np.max(np.abs(P - P_prev)) < eps:
            converged = True
            print('Converged after {} Ricatti iterations.'.format(i))
            break
        else:
            np.copyto(P_prev, P)

    if not converged:
        raise RuntimeError('Ricatti recursion did not converge!')
    
    return K
    
    