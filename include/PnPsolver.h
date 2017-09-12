/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{
/* *
 * Point n-Points EPnP solver.
 * Use RANSAC to find the pose of a keyframe that best explain a series of observed points, known position on the map.
 *
 * This class is instantiated repeatedly only in Tracking :: Relocalisation (), this context:
 * Several candidate keyframes are evaluated, presumably observing the same points observed by the current frame.
 * For each one the observed points with BoW are correlated, of the candidate keyframe its corresponding 3D coordinates are obtained,
 * And the solver is executed to get the pose that best explains the observation,
 *
 */
class PnPsolver {
 public:
	
  /*
   * Constructor, from a box and a 3D point vector.
   * - Initialize most of the attributes at zero.
   * - Assigns the definitive size to the vectors, and loads them with data from the frame F.
   * - Set Ransac parameters by default.
   * - Copy of F the intrinsic parameters.
   *
   * @param F Picture to relocalize.
   * @param vpMapPointMatches 3D points matched with a keyframe. It is stored in the PnPsolver :: mvpMapPointMatches attribute.
   *
   * Summoned only from Tracking :: Relocalization.
   */	
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();
   /* *
   * Set Ransac parameters.
   * All parameters have a default value, although orb-slam2 does not use them.
   *
   * @param probability Minimum probability of success accepted. This value is recorded in PnPsolver :: mRansacProb
   * @param minInliers Minimum amount of inliers accepted. This value is recorded in PnPsolver :: mRansacMinInliers
   * @param maxIterations Maximum amount of iterations allowed to find the solution. If he does, he gives up. This value is recorded in PnPsolver :: mRansacMaxIts
   * @param minSet Minimum number of points to consider in each iteration. This value is recorded in PnPsolver :: mRansacMinSet
   * @param epsilon This value is recorded in PnPsolver :: mRansacEpsilon
   * @param th2 Threshold to caudrado.
   *
   * Invoked with default parameters by the PnPsolver constructor, and by Tracking :: Relocalization with ad hoc parameters.
   *
   * In orb-slam2 every time an instance is built, this method is invoked twice consecutively:
   * First in the constructor with default values, and immediately after with ad hoc parameters.
   *
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

   /* *
   * Executes the maximum number of Ransac iterations.
   *
   * @param vbInliers Result, marks the position of the elements found inliers.
   * @param nInliers Restored, amount of found inliers.
   * @returns Pose the relocation, or an empty array if it does not fail.
   *
   * Not used in orb-slam2. PnPsolver :: iterate is used instead.
   */	
  cv::Mat find(vector<bool> &vbInliers, int &nInliers);
   /* *
   * Execute n iterations of Ransac.
   *
   * @param nIterations Number of iterations to execute. Finish earlier if PnPsolver :: mRansacMaxIts is reached in total.
   * @param bNoMore Result, sign indicating that the PnPsolver :: mRansacMaxIts were reached in total.
   * @param vbInliers Result, marks the position of the elements found inliers.
   * @param nInliers Restored, amount of found inliers.
   * @returns Pose the relocation, or an empty array if it does not fail.
   *
   * Used only in Tracking :: Relocalization. It is also invoked from the unused method find.
   */
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  /* * Reproject according to the calculated pose and redial as inliers only to those with an error less than maxError. */
  void CheckInliers();
  bool Refine();

  // Functions from the original EPnP code
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void print_pose(const double R[3][3], const double t[3]);
  double reprojection_error(const double R[3][3], const double t[3]);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];
  double cws_determinant;
  /* * 3D points macheados. */
  vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  vector<cv::Point2f> mvP2D;
  vector<float> mvSigma2;

  // 3D Points
  vector<cv::Point3f> mvP3Dw;

  // Index in Frame
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
