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

#ifndef UFO_VISION_COLOR_HPP
#define UFO_VISION_COLOR_HPP

// STL
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <utility>

namespace ufo
{
enum class ColorBlendingMode { MEAN, MEAN_ALPHA, SQ_MEAN, SQ_MEAN_ALPHA, NONE };

/*!
 * @brief Color
 *
 */
struct Color {
	using value_type = std::uint8_t;

	value_type alpha{std::numeric_limits<value_type>::max()};
	value_type blue{};
	value_type green{};
	value_type red{};

	constexpr Color() = default;

	constexpr Color(value_type v) : alpha(v), blue(v), green(v), red(v) {}

	constexpr Color(value_type v, value_type alpha)
	    : alpha(alpha), blue(v), green(v), red(v)
	{
	}

	constexpr Color(value_type red, value_type green, value_type blue,
	                value_type alpha = std::numeric_limits<value_type>::max())
	    : alpha(alpha), blue(blue), green(green), red(red)
	{
	}

	friend constexpr bool operator==(Color lhs, Color rhs)
	{
		return lhs.alpha == rhs.alpha && lhs.blue == rhs.blue && lhs.green == rhs.green &&
		       lhs.red == rhs.red;
	}

	friend constexpr bool operator!=(Color lhs, Color rhs) { return !(lhs == rhs); }

	[[nodiscard]] constexpr float redf() const noexcept
	{
		return red / static_cast<float>(std::numeric_limits<value_type>::max());
	}

	[[nodiscard]] constexpr float greenf() const noexcept
	{
		return green / static_cast<float>(std::numeric_limits<value_type>::max());
	}

	[[nodiscard]] constexpr float bluef() const noexcept
	{
		return blue / static_cast<float>(std::numeric_limits<value_type>::max());
	}

	[[nodiscard]] constexpr float alphaf() const noexcept
	{
		return alpha / static_cast<float>(std::numeric_limits<value_type>::max());
	}

	constexpr void redf(float v) noexcept
	{
		assert(0.0f <= v && 1.0f >= v);
		red = static_cast<value_type>(std::numeric_limits<value_type>::max() * v);
	}

	constexpr void greenf(float v) noexcept
	{
		assert(0.0f <= v && 1.0f >= v);
		green = static_cast<value_type>(std::numeric_limits<value_type>::max() * v);
	}

	constexpr void bluef(float v) noexcept
	{
		assert(0.0f <= v && 1.0f >= v);
		blue = static_cast<value_type>(std::numeric_limits<value_type>::max() * v);
	}

	constexpr void alphaf(float v) noexcept
	{
		assert(0.0f <= v && 1.0f >= v);
		alpha = static_cast<value_type>(std::numeric_limits<value_type>::max() * v);
	}

	void swap(Color& other) noexcept
	{
		std::swap(alpha, other.alpha);
		std::swap(blue, other.blue);
		std::swap(green, other.green);
		std::swap(red, other.red);
	}

	[[nodiscard]] constexpr inline bool empty() const noexcept { return 0 == alpha; }

	constexpr inline void clear() noexcept { alpha = blue = green = red = 0; }

	/*!
	 * @brief
	 *
	 * @note This works for up to 16,843,009 millions colors. If you have more
	 * then you should change to std::uint64_t
	 * @param first
	 * @param last
	 * @param blend_mode
	 * @return Color
	 */
	template <class InputIt>
	[[nodiscard]] static constexpr Color blend(InputIt first, InputIt last,
	                                           ColorBlendingMode blend_mode)
	{
		switch (blend_mode) {
			case ColorBlendingMode::MEAN: {
				unsigned alpha{};
				unsigned blue{};
				unsigned green{};
				unsigned red{};
				unsigned num{};
				for (; first != last; ++first) {
					Color c = *first;
					num += c.empty() ? 0u : 1u;
					alpha += static_cast<unsigned>(c.alpha);
					blue += static_cast<unsigned>(c.blue);
					green += static_cast<unsigned>(c.green);
					red += static_cast<unsigned>(c.red);
				}

				float f = num ? 1.0f / num : 0.0f;
				return {static_cast<Color::value_type>(f * red),
				        static_cast<Color::value_type>(f * green),
				        static_cast<Color::value_type>(f * blue),
				        static_cast<Color::value_type>(f * alpha)};
			}
			case ColorBlendingMode::MEAN_ALPHA: {
				unsigned alpha{};
				unsigned blue{};
				unsigned green{};
				unsigned red{};
				unsigned num{};
				for (; first != last; ++first) {
					Color c = *first;
					num += c.empty() ? 0u : 1u;
					unsigned a = static_cast<unsigned>(c.alpha);
					unsigned b = static_cast<unsigned>(c.blue);
					unsigned g = static_cast<unsigned>(c.green);
					unsigned r = static_cast<unsigned>(c.red);
					alpha += a;
					blue += a * b;
					green += a * g;
					red += a * r;
				}

				float f = num ? 1.0f / alpha : 0.0f;
				return {static_cast<Color::value_type>(f * red),
				        static_cast<Color::value_type>(f * green),
				        static_cast<Color::value_type>(f * blue),
				        static_cast<Color::value_type>(num ? alpha / num : 0)};
			}
			case ColorBlendingMode::SQ_MEAN: {
				unsigned alpha{};
				unsigned blue{};
				unsigned green{};
				unsigned red{};
				unsigned num{};
				for (; first != last; ++first) {
					Color c = *first;
					num += c.empty() ? 0u : 1u;
					unsigned a = static_cast<unsigned>(c.alpha);
					unsigned b = static_cast<unsigned>(c.blue);
					unsigned g = static_cast<unsigned>(c.green);
					unsigned r = static_cast<unsigned>(c.red);
					alpha += a;
					blue += b * b;
					green += g * g;
					red += r * r;
				}

				float f = num ? 1.0f / alpha : 0.0f;
				return {static_cast<Color::value_type>(std::sqrt(f * red)),
				        static_cast<Color::value_type>(std::sqrt(f * green)),
				        static_cast<Color::value_type>(std::sqrt(f * blue)),
				        static_cast<Color::value_type>(f * alpha)};
			}
			case ColorBlendingMode::SQ_MEAN_ALPHA: {
				unsigned alpha{};
				unsigned blue{};
				unsigned green{};
				unsigned red{};
				unsigned num{};
				for (; first != last; ++first) {
					Color c = *first;
					num += c.empty() ? 0u : 1u;
					unsigned a = static_cast<unsigned>(c.alpha);
					unsigned b = static_cast<unsigned>(c.blue);
					unsigned g = static_cast<unsigned>(c.green);
					unsigned r = static_cast<unsigned>(c.red);
					alpha += a;
					blue += a * b * b;
					green += a * g * g;
					red += a * r * r;
				}

				float f = num ? 1.0f / alpha : 0.0f;
				return {static_cast<Color::value_type>(std::sqrt(f * red)),
				        static_cast<Color::value_type>(std::sqrt(f * green)),
				        static_cast<Color::value_type>(std::sqrt(f * blue)),
				        static_cast<Color::value_type>(num ? alpha / num : 0)};
			}
			case ColorBlendingMode::NONE: return {};
		}

		return {};
	}

	/*!
	 * @brief
	 *
	 * @note This works for up to 16,843,009 millions colors. If you have more
	 * then you should change to std::uint64_t
	 * @param c
	 * @param blend_mode
	 * @return Color
	 */
	template <class Container>
	[[nodiscard]] static constexpr Color blend(Container const&  c,
	                                           ColorBlendingMode blend_mode)
	{
		using std::begin;
		using std::end;
		return blend(begin(c), end(c), blend_mode);
	}

	/*!
	 * @brief
	 *
	 * @note This works for up to 16,843,009 millions colors. If you have more
	 * then you should change to std::uint64_t
	 * @param ilist
	 * @param blend_mode
	 * @return Color
	 */
	[[nodiscard]] static constexpr Color blend(std::initializer_list<Color> ilist,
	                                           ColorBlendingMode            blend_mode)
	{
		using std::begin;
		using std::end;
		return blend(begin(ilist), end(ilist), blend_mode);
	}
};

inline std::ostream& operator<<(std::ostream& out, ufo::Color color)
{
	return out << "Red: " << +color.red << " Green: " << +color.green
	           << " Blue: " << +color.blue << " Alpha: " << +color.alpha;
}
}  // namespace ufo

#endif  // UFO_VISION_COLOR_HPP
