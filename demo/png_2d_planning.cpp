#include "png_2d_scenario.hpp"
#include <vector>
#include <png.h>
#include <cstdio>
#include <mpt/prrt_star.hpp>
#include "shape_hierarchy.hpp"

using namespace mpt_demo;
using namespace unc::robotics::mpt;
using namespace mpt_demo;
using namespace std::literals;
using namespace shape;

using Scalar = double;
using State = Eigen::Matrix<Scalar, 2, 1>;
using Algorithm = PRRTStar<>;
using Scenario = PNG2dScenario<Scalar>;

void filter(png_bytep *rowPointers, std::vector<PNGColor> &filters, int width, int height);
inline void writePngFile(png_bytep *rowPointers, int width, int height);

int main(int argc, char *argv[])
{
    /*
     * Read png file
     */
    std::string inputName = "../../png_planning_input.png";
    FILE *fp = std::fopen(inputName.c_str(), "rb");
    if(!fp) puts("file cannot be read");
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png) puts("png struct");
    png_infop info = png_create_info_struct(png);
    if(!info) puts("png info");
    if(setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);
    png_read_info(png, info);

    auto width = png_get_image_width(png, info);
    auto height = png_get_image_height(png, info);

    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth  = png_get_bit_depth(png, info);

    // std::cout << "bit depth: " << bit_depth << std::endl;
    // resolve pallete img to rgb
    if(color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);

    // restrict 1 byte per pixel
    if(bit_depth == 16)
        png_set_strip_16(png);
    if (bit_depth < 8)
        png_set_packing(png);

    // strip alpha channel
    if (color_type & PNG_COLOR_MASK_ALPHA)
        png_set_strip_alpha(png);
    // update the changes
    png_read_update_info(png, info);
    /*
     * allocate the bitmap
     */

    auto rowBytes = png_get_rowbytes(png, info);

    bit_depth  = png_get_bit_depth(png, info);
    std::cout << rowBytes / width << std::endl; // should print 3 (rgb)

    // std::cout << rowBytes / width << std::endl; // should print 3 (rgb)
    std::vector<png_bytep> rowPointers(height);
    std::vector<png_byte> image(rowBytes * height);
    for (int y = 0 ; y < height ; ++y)
        rowPointers[y] = &image[y * rowBytes];
    png_read_image(png, rowPointers.data());


    /*
     * filter the obstacle colors
     */

    std::vector<PNGColor> filters;
    filters.push_back(PNGColor(126, 106, 61));
    filters.push_back(PNGColor(61, 53, 6));
    filter(rowPointers.data(), filters, width, height);

    /*
     * Enable this to print a filtered image.
     */

    writePngFile(rowPointers.data(), width, height);

    /*
     * Initialize scenario and run planner
     */

    State startState, goalState;
    startState << 430, 1300;
    goalState << 3150, 950;


    Scenario scenario(width, height, goalState, rowPointers.data());

    static constexpr auto MAX_SOLVE_TIME = 10000ms;
    Planner<Scenario, Algorithm> planner(scenario);
    planner.addStart(startState);
    planner.solveFor(MAX_SOLVE_TIME);
    planner.printStats();

    /*
     * Draw the solution path and write it to a png
     */
    std::vector<State> solution = planner.solution();
    if(solution.empty())
    {
        MPT_LOG(INFO) <<  "No solution was found";
    }
    else
    {
        const std::string outputName = "png_2d_demo.svg";
        MPT_LOG(INFO) << "Writing to " << outputName;
        std::ofstream file(outputName);
        startSvg(file, width, height);
        addImage(file, inputName);

        if (!solution.empty())
        {
            for(auto it = solution.begin(); it + 1 != solution.end() ; ++it)
            {
                const auto &from = *it;
                const auto &to = *(it + 1);
                addSolutionEdge(file, from[0], from[1], to[0], to[1]);
            }
        }
        /*
         * Draw the visited links
         */
#if 1
        struct Visitor
        {
            std::ofstream &out_;
            State from_;

            Visitor(std::ofstream &out) : out_(out) {}

            void vertex(const State &q)
            {
                from_ = q;
            }

            void edge(const State &to)
            {
                static int count = 0;
                if(++count > 10000)
                    return;
                addVisitedEdge(out_, from_[0], from_[1], to[0], to[1]);
            }
        };
        planner.visitGraph(Visitor(file));
#endif
        endSvg(file);
    }

    /*
     * Free the png variables
     */
    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);
    return 0;
}

inline void writePngFile(png_bytep *rowPointers, int width, int height)
{
    const std::string outputName = "png_planning_filtered.svg";
    MPT_LOG(INFO) << "Writing to " << outputName;
    FILE *fp = fopen(outputName.c_str(), "wb");

    if(!fp) abort();

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) abort();

    png_infop info = png_create_info_struct(png);
    if (!info) abort();

    if (setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);

    // Output is 8bit depth, RGB format.
    png_set_IHDR(
        png,
        info,
        width, height,
        8,
        PNG_COLOR_TYPE_RGB,
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
            png_bytep px = &(row[x * 3]);
            bool isObstacle = false;
            if(px[0] > 250 && px[1] > 250 && px[2] > 250)
            {
                isObstacle = true;
            }
            else
            {
                for(auto const &c : filters)
                {
                    if(c.isObstacle(px[0], px[1], px[2], tolerance))
                    {
                        isObstacle = true;
                        break;
                    }
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