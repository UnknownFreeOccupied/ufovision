/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the
 * Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of
 * Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_VISION_CAMERA_HPP
#define UFO_VISION_CAMERA_HPP

// UFO
#include <ufo/geometry/shape/ray.hpp>
#include <ufo/math/pose3.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <cassert>
#include <cmath>

namespace ufo
{
/*!
 * @brief Camera
 *
 */
struct Camera {
	Pose3f pose;
	float  vertical_fov;
	float  near_clip;
	float  far_clip;
	Vec3f  up{0, 0, 1};
	Vec3f  right{0, -1, 0};
	Vec3f  forward{1, 0, 0};

	[[nodiscard]] Image<Ray3> rays(std::size_t rows, std::size_t cols) const
	{
		return rays(execution::seq, rows, cols);
	}

	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3> rays(ExecutionPolicy&& policy, std::size_t rows,
	                               std::size_t cols) const
	{
		float const tan_half_fovy = std::tan(vertical_fov / 2.0f);
		float const aspect        = cols / static_cast<float>(rows);

		Mat4x4f perspective(0);
		perspective[0][0] = 1.0f / (aspect * tan_half_fovy);
		perspective[1][1] = 1.0f / (tan_half_fovy);
		perspective[2][2] = far_clip / (far_clip - near_clip);
		perspective[2][3] = 1.0f;
		perspective[3][2] = -(far_clip * near_clip) / (far_clip - near_clip);

		auto perspective_inv = perspective;  // inverse(perspective);

		Mat4x4f view     = static_cast<Mat4x4f>(pose);
		auto    view_inv = view;  // inverse(view);

		Image<Ray3> rays(rows, cols, Ray3(pose.position, {}));

		auto fun = [&](std::size_t row) {
			auto r = ((row + 0.5f) / rows) * 2.0f - 1.0f;
			for (std::size_t col{}; col < cols; ++col) {
				auto  c = ((col + 0.5f) / cols) * 2.0f - 1.0f;
				Vec4f p_nds_h(r, c, -1.0f, 1.0f);
				auto  dir_eye            = perspective_inv * p_nds_h;
				dir_eye.w                = 0.0f;
				auto dir_world           = normalize(Vec3f(view_inv * dir_eye));
				rays(row, col).direction = dir_world;
			}
		};

		if constexpr (!std::is_same_v<execution::sequenced_policy,
		                              std::decay_t<ExecutionPolicy>>) {
#if defined(UFO_TBB)
			std::vector<std::size_t> indices(rows);
			std::iota(indices.begin(), indices.end(), 0);
			std::for_each(policy, indices.begin(), indices.end(), fun);
			return rays;
#elif defined(UFO_OMP)
#pragma omp parallel for
			for (std::size_t row = 0; row < rows; ++row) {
				fun(row);
			};
			return rays;
#endif
		}

		for (std::size_t row{}; row < rows; ++row) {
			fun(row);
		};

		return rays;
	}
};

// inline std::ostream &operator<<(std::ostream &out, ufo::Color color)
// {
// 	return out << "Red: " << +color.red << " Green: " << +color.green
// 	           << " Blue: " << +color.blue << " Alpha: " << +color.alpha;
// }
}  // namespace ufo

#endif  // UFO_VISION_CAMERA_HPP
