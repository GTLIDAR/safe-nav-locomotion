import numpy as np


def vis2d(phi, O):
# phi is the sdf of the scene
# O is the grid index of the vantage point

    psi = 1*phi

    # streamlined sweeping so we don't have to think about the indices
    for i in range(2):
        si = i*1 + i-1   # gives {-1 or 1}
        for j in range(2):
            sj = j*1 + j-1
            psi = sweep2d(phi, psi, O, si, sj)

    return psi


def compute_max_index(N, s, O):
# determine how many steps to take based on if sweeping forward or backwards
    if s == 1:
        indMax = N-O
    elif s == -1:
        indMax = O -(-1)
    return indMax


def sweep2d(phi, psi, O, si, sj):
# a way to write the sweep so that it is modular
# basically propagates information from the origin.
# si,sj,sk determine the direction of the indices and the sign of r

# need to turn off boundscheck for faster

    Ny = phi.shape[0]
    Nx = phi.shape[1]

    # compute indices
    I = compute_max_index(Ny, si, O[0])
    J = compute_max_index(Nx, sj, O[1])

    # a weird way to index the loop so that it can use c compatible range function
    for ri in range(I):
        i = O[0] + si*ri
        for rj in range(J):
            j = O[1] + sj*rj
            if (ri+rj !=0 ):  # this if should be removed but doesn't seem to affect much
                A = 1.0/(ri+rj)

                # clamp indices between 0 and N. The edges shouldn't matter because ri, rj would be zero if they fall off
                i_ = max(0, min(Ny-1, (i-si) ) ) 
                j_ = max(0, min(Nx-1, (j-sj) ) )
                psi[i,j] = A*(ri*psi[i_,j]+rj*psi[i,j_])
                psi[i,j] = min(phi[i,j],psi[i,j])

    return psi
