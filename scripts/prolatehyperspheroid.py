#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import random
import math
import numpy as np

class ProlateHyperspheroid:

        def uniformInBall(self, r, n):
            vec = np.empty(n)
            while True:
                for i in range(n):
                    vec[i] = random.uniform(-r, r)
                if np.linalg.norm(vec) <= r:
                    break
            return vec

        def updateRotation(self):
            # Mark the transform as out of date
            #self.isTransformUpToDate_ = False;
            # If the minTransverseDiameter_ is too close to 0, we treat this as a circle.
            circleTol = 1E-9;
            if self.minTransverseDiameter_ < circleTol:
                #rotationWorldFromEllipse_.setIdentity(dim_, dim_);
                rotationWorldFromEllipse_ = np.identity(self.dim_)
            else:
                # Variables
                # The transverse axis of the PHS expressed in the world frame.
                #Eigen::VectorXd transverseAxis(dim_);
                # The matrix representation of the Wahba problem
                #Eigen::MatrixXd wahbaProb(dim_, dim_);
                # The middle diagonal matrix in the SVD solution to the Wahba problem
                #Eigen::VectorXd middleM(dim_);

                # Calculate the major axis, storing as the first eigenvector
                transverseAxis = (self.xFocus2_ - self.xFocus1_ )/self.minTransverseDiameter_;

                # Calculate the rotation that will allow us to generate the remaining eigenvectors
                # Formulate as a Wahba problem, first forming the matrix a_j*a_i' where a_j is the transverse axis if the ellipse in the world frame, and a_i is the first basis vector of the world frame (i.e., [1 0 .... 0])
                #wahbaProb = transverseAxis*Eigen::MatrixXd::Identity(dim_, dim_).col(0).transpose();
                id_col0_inv = np.zeros(self.dim_)
                id_col0_inv[0] = 1
                wahbaProb = np.transpose(np.matrix(transverseAxis)) * id_col0_inv;
#                print "transverseAxis"
#                print transverseAxis
#                print "wahbaProb"
#                print wahbaProb

                # Then run it through the  SVD solver
                #Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd(wahbaProb, Eigen::ComputeFullV | Eigen::ComputeFullU);
                U, s, V = np.linalg.svd(wahbaProb, full_matrices=1, compute_uv=1)


                # Then calculate the rotation matrix from the U and V components of SVD
                # Calculate the middle diagonal matrix
                #middleM = Eigen::VectorXd::Ones(dim_);
                middleM = np.ones(self.dim_)
                # Make the last value equal to det(U)*det(V) (zero-based indexing remember)
                #middleM(dim_ - 1) = svd.matrixU().determinant()*svd.matrixV().determinant();
                middleM[self.dim_ - 1] = np.linalg.det(U) * np.linalg.det(V)

                # Calculate the rotation
                #rotationWorldFromEllipse_ = svd.matrixU()*middleM.asDiagonal()*svd.matrixV().transpose();
                self.rotationWorldFromEllipse_ = U * np.diag(middleM) * np.transpose(V)

        def __init__(self, n, focus1, focus2, minTransverseDiameter):
            self.transverseDiameter_ = 0.0
            self.dim_ = n
            self.xFocus1_ = focus1
            self.xFocus2_ = focus2
            self.minTransverseDiameter_ = minTransverseDiameter #np.linalg.norm(self.xFocus1_ - self.xFocus2_)
            self.xCentre_ = 0.5*(self.xFocus1_ + self.xFocus2_);
            self.updateRotation()

        def unitNBallMeasure(self, N):
#            return std::pow(std::sqrt(boost::math::constants::pi<double>()), static_cast<double>(N)) / boost::math::tgamma(static_cast<double>(N)/2.0 + 1.0);
            return pow(math.sqrt(math.pi), float(N)) / math.gamma(float(N)/2.0 + 1.0)

        def calcPhsMeasure(self, N, minTransverseDiameter, transverseDiameter):
            if transverseDiameter < minTransverseDiameter:
                print "Transverse diameter cannot be less than the minimum transverse diameter."
                exit(0)
            # Variable
            # The conjugate diameter:
            #double conjugateDiameter;
            # The Lebesgue measure return value
            #double lmeas;

            # Calculate the conjugate diameter:
            #conjugateDiameter = std::sqrt(transverseDiameter*transverseDiameter - minTransverseDiameter*minTransverseDiameter);
            conjugateDiameter = math.sqrt(transverseDiameter*transverseDiameter - minTransverseDiameter*minTransverseDiameter)

            # Calculate as a product series of the radii, noting that one is the transverse diameter/2.0, and the other N-1 are the conjugate diameter/2.0
            lmeas = transverseDiameter/2.0;
            for i in range(1, N): #(unsigned int i = 1u; i < N; ++i)
                lmeas = lmeas * conjugateDiameter/2.0;

            # Then multiplied by the volume of the unit n-ball.
            lmeas = lmeas * self.unitNBallMeasure(N);

            # Return:
            return lmeas;

        def updateTransformation(self):
            # Variables
            # The radii of the ellipse
            #Eigen::VectorXd diagAsVector(dim_);
            diagAsVector = np.empty(self.dim_)
            # The conjugate diameters:
            #double conjugateDiamater;

            # Calculate the conjugate radius
            #conjugateDiamater = std::sqrt(transverseDiameter_*transverseDiameter_ - minTransverseDiameter_*minTransverseDiameter_);
            conjugateDiamater = math.sqrt(self.transverseDiameter_*self.transverseDiameter_ - self.minTransverseDiameter_*self.minTransverseDiameter_)

            # Store into the diagonal matrix
            # All the elements but one are the conjugate radius
            diagAsVector.fill(conjugateDiamater/2.0);

            # The first element in diagonal is the transverse radius
            #diagAsVector(0) = 0.5*transverseDiameter_;
            diagAsVector[0] = 0.5*self.transverseDiameter_

            # Calculate the transformation matrix
            #transformationWorldFromEllipse_ = rotationWorldFromEllipse_*diagAsVector.asDiagonal();
            self.transformationWorldFromEllipse_ = self.rotationWorldFromEllipse_*np.diag(diagAsVector);

            # Calculate the measure:
            #phsMeasure_ = calcPhsMeasure(dim_, minTransverseDiameter_, transverseDiameter_);
            self.phsMeasure_ = self.calcPhsMeasure(self.dim_, self.minTransverseDiameter_, self.transverseDiameter_);

            # Mark as up to date
            #isTransformUpToDate_ = true;

        def transform(self, sphere):
            #if (isTransformUpToDate_ == false)
            #    throw Exception("The transformation is not up to date in the PHS class. Has the transverse diameter been set?");

            # Calculate the tranformation and offset, using Eigen::Map views of the data
            #Eigen::Map<Eigen::VectorXd>(phs, n) = transformationWorldFromEllipse_*Eigen::Map<const Eigen::VectorXd>(sphere, n) + xCentre_;
#            print "self.transformationWorldFromEllipse_"
#            print self.transformationWorldFromEllipse_
#            print "sphere"
#            print sphere
#            print "self.xCentre_"
#            print self.xCentre_
            return self.transformationWorldFromEllipse_ * np.transpose(np.matrix(sphere)) + np.transpose(np.matrix(self.xCentre_));

        def setTransverseDiameter(self, transverseDiameter):
            if transverseDiameter+0.001 < self.minTransverseDiameter_:
                print "ERROR: setTransverseDiameter:", transverseDiameter, " <", self.minTransverseDiameter_
            #    std::cout << transverseDiameter << " < " << minTransverseDiameter_ << std::endl;
            #    throw Exception("Transverse diameter cannot be less than the distance between the foci.");

            # Store and update if changed
#            if self.transverseDiameter_ != transverseDiameter:
                # Mark as out of date
                #isTransformUpToDate_ = false;

            # Store
            self.transverseDiameter_ = transverseDiameter+0.001

            # Update the transform
            self.updateTransformation()
            # No else, the diameter didn't change

