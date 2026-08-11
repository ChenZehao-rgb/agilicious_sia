import numpy as np


def compute_A_matrix(continuity_order=4):
    A = np.zeros((2 * continuity_order + 1, 2 * continuity_order + 1))
    # enforce no offset
    A[0, -1] = 1.0
    p = np.poly1d(np.ones(2 * continuity_order + 1))
    # derivative constraints
    for i_der in range(2 * continuity_order):
        for i_coeff in range(2 * continuity_order + 1):
            # differentiate between start and end constraint
            if i_der % 2 == 0:
                # single coefficient is nonzero
                A[i_der + 1, 2 * continuity_order - i_der // 2 - 1] = np.polyder(p, i_der // 2 + 1).coeffs[-1]
            else:
                A[i_der + 1, :np.polyder(p, (i_der + 1) // 2).coeffs.shape[0]] = np.polyder(p, (i_der + 1) // 2).coeffs
    return A


def time_mapping(speedup_duration, dt, continuity_order=4):
    t_vec, dt = np.linspace(0.0, speedup_duration,
                            int(speedup_duration / dt),
                            endpoint=False, retstep=True)

    x = time_mapping_poly_coeffs(continuity_order)

    y_vec = np.zeros_like(t_vec)
    for i in range(2 * continuity_order + 1):
        y_vec += speedup_duration * x[i] * np.power(t_vec / speedup_duration, 2 * continuity_order - i)

    return y_vec


def time_mapping_poly_coeffs(continuity_order=4):
    assert continuity_order >= 1
    A = compute_A_matrix(continuity_order)
    b = np.zeros(2 * continuity_order + 1)
    b[2] = 1.0  # we want to reach 'real-time' at the end of the window
    x = np.linalg.solve(A, b)

    return x
