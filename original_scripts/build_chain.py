"""

Author(s): Mateo Patino, Hod Lipson
Creative Machines Laboratory, Columbia University

This script builds a chain or enclosure for the particles using link objects in MuJoCo. It also
provides functions that dynamically adjust link length to achieve the desired chain tightness and
offset the chain's initial position to avoid interpenetration.

"""
        
import lowsizeparams as params
import math
import numpy as np

""" 
    Calculates the length L of one chain link given a tightness ratio, area spanned by the particles, number of links per side
    and the radius of the capsules. This formula was derived by hand by finding the side length spanned by
    the square arrangement of particles and the side length of the inner edge of the chain.
"""
def linkLength(tau, particleArea, linksPerSide, capsuleRadius):
    L = (1 / linksPerSide) * math.sqrt(particleArea / tau) + (2 * capsuleRadius / linksPerSide)
    print("\nLink length (m)", L)
    return L


"""
    Calculates the distance by which to offset the bottom-left chain link from the origin. This offset is very important
    to avoid interpenetration between the particles and the chain before the simulation begins, which could create large
    initial forces that break the chain or send particles flying off to infinity. This formula was derived by hand by 
    finding the distance between the top-right particle and the top-right chain link if the bottom-left chain link was 
    placed exactly on top of the bottom-left particle. The offset distance is half of that distance and is always a
    negative number.
"""
def chainOffsetFromOrigin(particlesPerSide, restSeparation, linksPerSide, L):
    return ((particlesPerSide - 1) * restSeparation - linksPerSide * L) / 2


# returns the XML string for a square chain with links on each side of length L (tightness-dependent)
def _createChainXML_(linksPerSide, tau):

    # calculate the length of each link according to the desired tightness
    particleArea = params.N * np.pi * pow(params.particle["radius"], 2)
    L = linkLength(tau, particleArea, linksPerSide, params.capsule["radius"])

    capsuleBodies = horizontalLinks(linksPerSide, L, 1, 1)

    particlesPerSide = math.ceil(math.sqrt(params.N))
    restSeparation = 2 * params.particle["radius"] + params.clearance
    offset = chainOffsetFromOrigin(particlesPerSide, restSeparation, linksPerSide, L)

    # set the first link in the chain manually. Create the rest of the links recursively
    chain = f""" 
        <body name="capsule1" pos="{offset} {offset} {params.capsule["radius"]}">
            <freejoint/>
            <geom class="link" fromto="0 0 0 {L} 0 0"/>
            <site name="start_anchor" size="0.001" pos="0 0 0"/>

            {capsuleBodies}

        </body>
    """

    return chain

# n = number of links per side, L = length of one link
def horizontalLinks(n, L, count, total):

    # base case: last link in the bottom side of the chain
    if count == n:
        return verticalLinks(n, L, 0, total)
    
    # if it is the first horizontal link at the top, put it at (0, L, 0), else at (L, 0, 0)
    if count == 0:
        # we negate L because L is negative when the call to start the top side is made
        return f""" 
            <body name="capsule{total + 1}" pos="0 {-1 * L} 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/>

                {horizontalLinks(n, L, count + 1, total + 1)}

            </body>
        """
    else:
        return f""" 
            <body name="capsule{total + 1}" pos="{L} 0 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 {L} 0 0"/>

                {horizontalLinks(n, L, count + 1, total + 1)}

            </body>
        """


def verticalLinks(n, L, count, total):

    # base case 1: stop the recursion and add the site for closing the loop
    if count == n and L < 0:
        return f"""  
            <site name="end_anchor" size="0.001" pos="0 {L} 0"/>
        """
    # base case 2: the right side of the chain is complete, begin the top side
    elif count == n and L > 0:
        return horizontalLinks(n, -L, 0, total)
        
    # if it is the first vertical link, put body L units to the right or left of parent, which is the last horizontal link
    if count == 0:
        return f"""
            <body name="capsule{total + 1}" pos="{L} 0 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 0 {L} 0"/>

                {verticalLinks(n, L, count + 1, total + 1)}

            </body>
        """
    # else place the body L units above or below
    else:
        return f"""
            <body name="capsule{total + 1}" pos="0 {L} 0">
                <joint type="hinge" axis="0 0 1" pos="0 0 0"/>
                <geom class="link" fromto="0 0 0 0 {L} 0"/>

                {verticalLinks(n, L, count + 1, total + 1)}

            </body>
        """

