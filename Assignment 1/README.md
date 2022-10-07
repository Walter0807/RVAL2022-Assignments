# RVAL 2022 Assignment 1

In this assignment, you will create a simple toolbox `myquaternion` for handling rotations using `numpy`. 

## Functions to implement

Detailed descriptions of each function can be found in `myquaternion.py`

### Basic Operations

1. `normalize()`
2. `multiply()`
3. `conjugate()`
4. `rotate()`
5. `relative_angle()`
6. `interpolate_quaternions()`

### Conversion

1. `quaternion_to_matrix()`
2. `matrix_to_quaternion()`
3. `quaternion_to_rotvec()`
4. `rotvec_to_quaternion()`
5. `rotvec_to_matrix()`
6. `matrix_to_rotvec()`

### Random Sampling

1. `generate_random_quaternion()`

## Files

- `myquaternion.py`: Your rotation processing libray. You need to implement all the functions in it.
- `eval_myquaternion.py`: Scoring script for self-evaluation.
- `eval_data.pkl`: Validation set used by the scoring script.

## Grading
- The assignment will be evaluated by running `eval_myquaternion.py` on a held-out test set to check the correctness. If you only manage to achieve part of the objectives, you will receive partial score. 
- It is not necessary to import extra libraries. You will also lose points if you use extra libraries like `scipy` and `transform3d` (*i.e.* you need to write the calculations by yourself). Late submission will also lose points. 

## Turning it in
- The deadline of assignment 1 is October 22nd, 12 p.m.
- Submit `myquaternion.py` and a (very simple) PDF document with self-evaluation results in a single `.zip` file to the [school course website](http://course.pku.edu.cn).

## Hints

> - We use the $(w, x, y, z)$ convention for quaternions (as in the slides).
> - If you can not pass some test data, you can just check them with the scoring script.
> - Think about the singularity of your functions.
> - If you have questions, please post them to the discussion board on the [school course website](http://course.pku.edu.cn).

