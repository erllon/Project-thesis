import numpy as np

def get_index_combs(n_indices, n_choices):
  if n_choices > n_indices:
    return []
  a = np.ones((n_choices, n_indices-n_choices+1), dtype=int)
  a[0] = np.arange(n_indices-n_choices+1)
  for j in range(1, n_choices):
      reps = (n_indices-n_choices+j) - a[j-1]
      a = np.repeat(a, reps, axis=1)
      ind = np.add.accumulate(reps)
      a[j, ind[:-1]] = 1-reps[1:]
      a[j, 0] = j
      a[j] = np.add.accumulate(a[j])
  return a.T

if __name__ == "__main__":
  num_agents = 0
  N = np.linspace(0, num_agents-1, num_agents)
  for i in range(num_agents):
    B = N[:i]
    C = N[i+1:]

    lN, = N.shape
    lB, = B.shape
    lC, = C.shape

    N_3_combs = N[get_index_combs(lN, 3)]
    counter = N_3_combs.shape[0]
    combs = set([tuple(sorted(comb)) for comb in N_3_combs])

    B_3_combs = B[get_index_combs(lB, 3)].reshape(-1, 3)
    counter -= B_3_combs.shape[0]
    for comb in B_3_combs:
      combs.remove(tuple(sorted(comb)))

    B_2_combs = B[get_index_combs(lB, 2)].reshape(-1, 2)

    B_2_C_combs = np.hstack((np.repeat(B_2_combs, lC, axis=0), np.tile(C.reshape(-1, 1), (int((1/2)*(lB-1)*lB), 1))))
    counter -= B_2_C_combs.shape[0]
    for comb in B_2_C_combs:
      combs.remove(tuple(sorted(comb)))

    B_2_i_combs = np.hstack((B_2_combs, i*np.ones((int((1/2)*(lB-1)*lB), 1))))
    counter -= B_2_i_combs.shape[0]
    for comb in B_2_i_combs:
      combs.remove(tuple(sorted(comb)))

    C_3_combs = C[get_index_combs(lC, 3)].reshape(-1, 3)
    counter -= C_3_combs.shape[0]
    for comb in C_3_combs:
      combs.remove(tuple(sorted(comb)))

    C_2_combs = C[get_index_combs(lC, 2)].reshape(-1, 2)

    C_2_B_combs = np.hstack((np.repeat(C_2_combs, lB, axis=0), np.tile(B.reshape(-1, 1), (int((1/2)*(lC-1)*lC), 1))))
    counter -= C_2_B_combs.shape[0]
    for comb in C_2_B_combs:
      combs.remove(tuple(sorted(comb)))

    C_2_i_combs = np.hstack((C_2_combs, i*np.ones((int((1/2)*(lC-1)*lC), 1))))
    counter -= C_2_i_combs.shape[0]
    for comb in C_2_i_combs:
      combs.remove(tuple(sorted(comb)))

    B_C_i_combs = np.hstack((np.repeat(B.reshape(-1, 1), lC, axis = 0), np.tile(C.reshape(-1, 1), (lB, 1)), i*np.ones((lB*lC, 1))))
    counter -= B_C_i_combs.shape[0]
    for comb in B_C_i_combs:
      combs.remove(tuple(sorted(comb)))

    assert counter == 0 and len(combs) == 0
    print(f"All GUCCI {i}")

