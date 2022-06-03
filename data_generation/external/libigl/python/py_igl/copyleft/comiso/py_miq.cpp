// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2017 Sebastian Koch <s.koch@tu-berlin.de> and Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
m.def("miq", []
(
  const Eigen::MatrixXd &V,
  const Eigen::MatrixXi &F,
  const Eigen::MatrixXd &PD1,
  const Eigen::MatrixXd &PD2,
  Eigen::MatrixXd &UV,
  Eigen::MatrixXi &FUV,
  double scale,
  double stiffness,
  bool directRound,
  int iter,
  int localIter,
  bool doRound,
  bool singularityRound
)
{
  std::vector<int> roundVertices;
  std::vector<std::vector<int> > hardFeatures;

  igl::copyleft::comiso::miq(V, F, PD1, PD2, UV, FUV, scale, stiffness, directRound, iter, localIter, doRound, singularityRound, roundVertices, hardFeatures);
}, __doc_igl_copyleft_comiso_miq,
py::arg("V"), py::arg("F"), py::arg("PD1"), py::arg("PD2"), py::arg("UV"), py::arg("FUV"), py::arg("scale") = 30.0, py::arg("stiffness") = 5.0, py::arg("directRound") = false, py::arg("iter") = 5, py::arg("localIter") = 5, py::arg("doRound") = true, py::arg("singularityRound") = true
);

m.def("miq", []
(
  const Eigen::MatrixXd &V,
  const Eigen::MatrixXi &F,
  const Eigen::MatrixXd &PD1_combed,
  const Eigen::MatrixXd &PD2_combed,
  const Eigen::MatrixXi &mismatch,
  const Eigen::MatrixXi &singular,
  const Eigen::MatrixXi &seams,
  Eigen::MatrixXd &UV,
  Eigen::MatrixXi &FUV,
  double gradientSize,
  double stiffness,
  bool directRound,
  int iter,
  int localIter,
  bool doRound,
  bool singularityRound
)
{
  assert_is_VectorX("singular",singular);

  std::vector<int> roundVertices;
  std::vector<std::vector<int> > hardFeatures;

  igl::copyleft::comiso::miq(V, F, PD1_combed, PD2_combed, mismatch, singular, seams, UV, FUV, gradientSize, stiffness, directRound, iter, localIter, doRound, singularityRound, roundVertices, hardFeatures);
}, __doc_igl_copyleft_comiso_miq,
py::arg("V"), py::arg("F"), py::arg("PD1_combed"), py::arg("PD2_combed"),
py::arg("mismatch"), py::arg("singular"), py::arg("seams"),
py::arg("UV"), py::arg("FUV"), py::arg("gradientSize") = 30.0, py::arg("stiffness") = 5.0, py::arg("directRound") = false, py::arg("iter") = 5, py::arg("localIter") = 5, py::arg("doRound") = true, py::arg("singularityRound") = true
);
