import numpy as np
import numpy.testing as npt
import myquaternion as q
import pickle

num_test = 20
rtol = 1e-4
score = 0

with open('eval_data.pkl', 'rb') as pickle_file:
    eval_data = pickle.load(pickle_file)

for k in range(num_test):
    print('Test [{}/{}]'.format(k, num_test))
    x, y, v, r, x_gt, y_gt, qmul_gt, xc_gt, v_rotate_gt, angle_gt, matrix_gt, rotvec_gt, i_gt = eval_data[k]
    try:
        # Check normalize()
        x = q.normalize(x)
        y = q.normalize(y)
        npt.assert_allclose(x, x_gt, rtol=rtol)
        npt.assert_allclose(y, y_gt, rtol=rtol)
        print('Pass test: normalize() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: normalize().')
        print(e)

    try:
        # Check multiply:
        qmul = q.multiply(x, y)
        npt.assert_allclose(qmul, qmul_gt, rtol=rtol)
        print('Pass test: multiply() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: multiply().')
        print(e)

    try:
        # Check conjugate:
        xc = q.conjugate(x)
        npt.assert_allclose(xc, xc_gt, rtol=rtol)
        print('Pass test: conjugate() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: conjugate().')
        print(e)

    try:
        # Check rotate
        v_rotate = q.rotate(x, v)
        npt.assert_allclose(v_rotate, v_rotate_gt, rtol=rtol)
        print('Pass test: rotate() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: rotate().')
        print(e)

    try:
        # Check relative_angle
        angle = q.relative_angle(x, y)
        angle_p = q.relative_angle(-x, y)
        npt.assert_allclose(angle, angle_gt, rtol=rtol)
        npt.assert_allclose(angle_p, angle_gt, rtol=rtol)
        print('Pass test: relative_angle() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: relative_angle().')
        print(e)

    try:
        # Check quaternion_to_matrix
        matrix = q.quaternion_to_matrix(x)
        npt.assert_allclose(matrix, matrix_gt, rtol=rtol)
        print('Pass test: quaternion_to_matrix() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: quaternion_to_matrix().')
        print(e)

    try:
        # Check matrix_to_quaternion
        x_h = q.matrix_to_quaternion(matrix_gt)
        assert (np.allclose(x_h, x_gt, rtol=rtol) or np.allclose(x_h, -x_gt, rtol=rtol))
        print('Pass test: matrix_to_quaternion() with rtol: {}.'.format(rtol))
        score += 1
    except Exception as e:
        print('Fail test: matrix_to_quaternion().')
        print(e) 

    try:
        # Check quaternion_to_rotvec
        rotvec = q.quaternion_to_rotvec(x)
        npt.assert_allclose(rotvec, rotvec_gt, rtol=rtol)
        print('Pass test: quaternion_to_rotvec() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: quaternion_to_rotvec().')
        print(e) 

    try:
        # Check rotvec_to_quaternion
        x_h = q.rotvec_to_quaternion(rotvec_gt)
        assert (np.allclose(x_h, x_gt, rtol=rtol) or np.allclose(x_h, -x_gt, rtol=rtol))
        print('Pass test: rotvec_to_quaternion() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: rotvec_to_quaternion().')
        print(e) 

    try:
        # Check rotvec_to_matrix
        mat_h = q.rotvec_to_matrix(rotvec_gt)
        npt.assert_allclose(mat_h, matrix_gt, rtol=rtol)
        print('Pass test: rotvec_to_matrix() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: rotvec_to_matrix().')
        print(e) 

    try:
        # Check matrix_to_rotvec
        rotvec_h = q.matrix_to_rotvec(matrix_gt)
        npt.assert_allclose(rotvec_h, rotvec_gt, rtol=rtol)
        print('Pass test: matrix_to_rotvec() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: matrix_to_rotvec().')
        print(e) 

    try:
        # Check interpolation
        i = q.interpolate_quaternions(x, y, ratio=r)
        npt.assert_allclose(i, i_gt, rtol=rtol)
        print('Pass test: interpolate_quaternions() with rtol: {}.'.format(rtol))
        score += 1 
    except Exception as e:
        print('Fail test: interpolate_quaternions().')
        print(e) 

    print('\n')

# Check random sampling with theoretical distribution statistics.
N = 10000
anchor = np.array([1,0,0,0])
thres = 1e-2
alphas = [np.pi/6, np.pi/3, np.pi/2]
try:
    for idx, alpha in enumerate(alphas):
        cnt = 0
        for i in range(N):
            sample = q.generate_random_quaternion()
            dist = q.relative_angle(sample, anchor)
            if abs(dist) <= alpha:
                cnt += 1
        oracle = (alpha - np.sin(alpha)) / np.pi
        if abs(cnt/N - oracle) <= thres:
            score += num_test
            print('Pass sampling case %d/%d' % (idx, len(alphas)))
        else:
            print('Fail sampling case %d/%d' % (idx, len(alphas)))
except Exception as e:
    print('Fail test: Random Sampling.')
    print(e) 

print('Overall Score = %d/%d' % (score, num_test*(12+len(alphas))))










