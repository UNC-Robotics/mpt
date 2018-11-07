#include "png_2d_scenario.hpp"
#include <vector>
#include <png.h>
#include <cstdio>
#include <mpt/prrt_star.hpp>

using namespace mpt_demo;
using namespace unc::robotics::mpt;
using namespace mpt_demo;
using namespace std::literals;

using Scalar = double;
using State = Eigen::Matrix<Scalar, 2, 1>;
using Algorithm = PRRTStar<>;
using Scenario = PNG2dScenario<Scalar>;

void filter(png_bytep *rowPointers, std::vector<PNGColor> &filters, int width, int height);
void writePngFile(png_bytep *rowPointers, int width, int height);
void fillLine(png_bytep *rowPointers, const State &from, const  State &to);
void fillMid(png_bytep *rowPointers, const State &from, const  State &to);

int main(int argc, char *argv[])
{
    /*
     * Read png file
     */
    int width;
    int height;

    std::string inputName = "../../input.png";
    FILE *fp = std::fopen(inputName.c_str(), "rb");
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);
    if(setjmp(png_jmpbuf(png))) abort();
    png_init_io(png, fp);
    png_read_info(png, info);

    width = png_get_image_width(png, info);
    height = png_get_image_height(png, info);

    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth  = png_get_bit_depth(png, info);

    if(bit_depth == 16)
        png_set_strip_16(png);
    if(color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);
    // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
    if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
        png_set_expand_gray_1_2_4_to_8(png);
    if(png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png);
    // These color_type don't have an alpha channel then fill it with 0xff.
    if(color_type == PNG_COLOR_TYPE_RGB ||
            color_type == PNG_COLOR_TYPE_GRAY ||
            color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
    if(color_type == PNG_COLOR_TYPE_GRAY ||
            color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);

    /*
     * allocate the bitmap
     */

    png_bytep *rowPointers;
    rowPointers = (png_bytep *)malloc(sizeof(png_bytep) * height);
    for(int y = 0; y < height; y++)
        rowPointers[y] = (png_byte *)malloc(png_get_rowbytes(png, info));
    png_read_image(png, rowPointers);

    /*
     * filter the obstacle colors
     */

    std::vector<PNGColor> filters;
    filters.push_back(PNGColor(130, 118, 103));
    filters.push_back(PNGColor(144, 129, 106));
    filter(rowPointers, filters, width, height);

    fclose(fp);

    /*
     * Initialize scenario and run planner
     */

    State startState, goalState;
    startState << 153, 194;
    goalState << 840, 380;


    Scenario scenario(width, height, goalState, rowPointers);

    static constexpr auto MAX_SOLVE_TIME = 500ms;
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(startState);
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();

    /*
     * Draw the solution path and write it to a png
     */

    std::vector<State> solution = planner.solution();
    if (!solution.empty())
    {
        for(auto it = solution.begin(); it + 1 != solution.end() ; ++it)
        {
            const auto &from = *it;
            const auto &to = *(it + 1);
            fillLine(rowPointers, from, to);
        }
    }

    writePngFile(rowPointers, width, height);
    /*
     * Free the bitmap pointers
     */
    for(int y = 0; y < height; y++)
        free(rowPointers[y]);
    free(rowPointers);
    return 0;
}

inline void fillLine(png_bytep *rowPointers, const State &from, const  State &to)
{
    int x = (int) (from[0] + 0.5);
    int y = (int) (from[1] + 0.5);
    png_bytep row = rowPointers[y];
    png_bytep px = &(row[x * 4]);

    px[0] = 255;
    px[1] = 50;
    px[2] = 50;

    x = (int) (to[0] + 0.5);
    y = (int) (to[1] + 0.5);
    row = rowPointers[y];
    px = &(row[x * 4]);

    px[0] = 255;
    px[1] = 50;
    px[2] = 50;

    fillMid(rowPointers, from, to);
}

void fillMid(png_bytep *rowPointers, const State &a, const  State &b)
{
    State mid = (a + b) / 2;
    Scalar distSquared = (b - a).squaredNorm();
    Scalar tolerance = 1;

    int x = (int) (mid[0] + 0.5);
    int y = (int) (mid[1] + 0.5);
    png_bytep row = rowPointers[y];
    png_bytep px = &(row[x * 4]);
    px[0] = 255;
    px[1] = 50;
    px[2] = 50;
    if(distSquared < tolerance * tolerance)
        return;
    fillMid(rowPointers, a, mid);
    fillMid(rowPointers, mid, b);
}

inline void writePngFile(png_bytep *rowPointers, int width, int height)
{
    FILE *fp = fopen("output2.png", "wb");
    if(!fp) abort();

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) abort();

    png_infop info = png_create_info_struct(png);
    if (!info) abort();

    if (setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);

    // Output is 8bit depth, RGBA format.
    png_set_IHDR(
        png,
        info,
        width, height,
        8,
        PNG_COLOR_TYPE_RGBA,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
    );
    png_write_info(png, info);

    // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
    // Use png_set_filler().
    //png_set_filler(png, 0, PNG_FILLER_AFTER);

    png_write_image(png, rowPointers);
    png_write_end(png, NULL);
    fclose(fp);
}

inline void filter(png_bytep *rowPointers, std::vector<PNGColor> &filters, int width, int height)
{
    const int tolerance = 15;
    for(int y = 0; y < height; y++)
    {
        png_bytep row = rowPointers[y];
        for(int x = 0; x < width; x++)
        {
            png_bytep px = &(row[x * 4]);
            bool isObstacle = false;
            for(auto const &c : filters)
            {
                if(c.isObstacle(px[0], px[1], px[2], tolerance))
                {
                    isObstacle = 1;
                    break;
                }
            }
            // TODO: Track invalid pixel independently. e.g. make isValid[x][y] matrix
            if(isObstacle)
            {
                px[0] = 0;
                px[1] = 0;
                px[2] = 0;
            }
            else
            {
                px[0] = 255;
                px[1] = 255;
                px[2] = 255;
            }
        }
    }
}