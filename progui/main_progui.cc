#include "progui_globals.h"
#include <cstdio>
#include <cstring>
#include <iostream>

int main(int argc, const char **argv)
{
    // build model procedurally: no XML, create world + floor
    spec = mj_makeSpec();
    if (!spec)
    {
        mju_error("Could not create mjSpec");
    }

    // add a floor plane to the world with a checker texture for tiling
    mjsBody* world = mjs_findBody(spec, "world");
    if (!world)
    {
        mju_error("World body not found");
    }
    {
        // texture (medium gray + dark blue)
        mjsTexture* tex = mjs_addTexture(spec);
        mjs_setName(tex->element, "floor_tex");
        tex->type = mjTEXTURE_2D;
        tex->builtin = mjBUILTIN_CHECKER;
        tex->mark = mjMARK_EDGE;
        // medium gray
        tex->rgb1[0] =0.55; tex->rgb1[1] =0.55; tex->rgb1[2] =0.55;
        // dark blue
        tex->rgb2[0] =0.06; tex->rgb2[1] =0.10; tex->rgb2[2] =0.35;
        tex->markrgb[0] =0.10; tex->markrgb[1] =0.10; tex->markrgb[2] =0.10;
        tex->width =512;
        tex->height =512;
        tex->nchannel =3;

        // material
        mjsMaterial* mat = mjs_addMaterial(spec, nullptr);
        mjs_setName(mat->element, "floor_mat");
        mat->texrepeat[0] =12.0f; // tile many times so it's easy to see
        mat->texrepeat[1] =12.0f;
        // bind the RGB texture
        mjs_setInStringVec(mat->textures, mjTEXROLE_RGB, "floor_tex");

        // floor geom
        mjsGeom* floor = mjs_addGeom(world, nullptr);
        floor->type = mjGEOM_PLANE;
        floor->size[0] =25.0; // plane extents (visualization)
        floor->size[1] =25.0;
        floor->size[2] =0.1;
        // assign material with checker texture (keep white so texture shows as-is)
        mjs_setString(floor->material, "floor_mat");
        floor->rgba[0] = 0.5f;
        floor->rgba[1] = 0.5f;
        floor->rgba[2] = 0.5f;
        floor->rgba[3] = 0.5f;
        floor->contype =1;
        floor->conaffinity =1 |2 |4;
        floor->friction[0] =1.0;
        floor->friction[1] =0.005;
        floor->friction[2] =0.0001;
        floor->margin = kGeomMargin;
        floor->solref[0] = kSolref[0];
        floor->solref[1] = kSolref[1];
        for (int i =0; i <5; ++i) floor->solimp[i] = kSolimp[i];

        // lighting: use only supported directional lights
        auto add_directional_light = [&](double dx, double dy, double dz,
                                         float diff, float spec, float amb){
            mjsLight* L = mjs_addLight(world, nullptr);
            L->type = mjLIGHT_DIRECTIONAL; // supported type
            L->active =1;
            L->dir[0] = dx; L->dir[1] = dy; L->dir[2] = dz;
            L->castshadow =1;
            L->diffuse[0] = diff; L->diffuse[1] = diff; L->diffuse[2] = diff;
            L->specular[0] = spec; L->specular[1] = spec; L->specular[2] = spec;
            L->ambient[0] = amb; L->ambient[1] = amb; L->ambient[2] = amb;
        };

        // two opposing directional lights from above for even illumination
        add_directional_light(-0.3, -0.2, -1.0,1.0f,0.6f,0.06f);
        add_directional_light(0.3,0.2, -1.0,0.8f,0.4f,0.04f);
    }

    // compile model and allocate data
    m = mj_compile(spec, nullptr);
    if (!m)
    {
        mju_error("Could not compile model: %s", mjs_getError(spec));
    }
    d = mj_makeData(m);

    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    GLFWwindow *window = glfwCreateWindow(1200,900, "ProGUI - Cube Spawner", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // initialize chain state and disable gravity initially
    g_lastBody = nullptr;
    g_chain.clear();

    g_savedGravity[0] = m->opt.gravity[0];
    g_savedGravity[1] = m->opt.gravity[1];
    g_savedGravity[2] = m->opt.gravity[2];

    m->opt.gravity[0] =0.0;
    m->opt.gravity[1] =0.0;
    m->opt.gravity[2] =0.0;
    mj_forward(m, d);

    mjv_makeScene(m, &scn,2000);
    // use smaller font scale for UI elements
    mjr_makeContext(m, &con, mjFONTSCALE_100);

    std::memset(&uistate,0, sizeof(mjuiState));
    std::memset(&ui,0, sizeof(mjUI));
    ui.spacing = mjui_themeSpacing(1);
    ui.color = mjui_themeColor(1);
    ui.predicate = nullptr;
    ui.rectid =1;
    ui.auxid =0;
    // render radio groups in single column; button pairing is disabled in ui_main, but set radiocol=1 for consistency
    ui.radiocol =1;
    ui.color.button[0] =0.15f;
    ui.color.button[1] =0.55f;
    ui.color.button[2] =0.95f;

    // Build defControl with global button names
    mjuiDef defControl[12];
    int di =0;

    // Section
    std::memset(&defControl[di],0, sizeof(mjuiDef));
    defControl[di].type = mjITEM_SECTION;
    std::snprintf(defControl[di].name, sizeof(defControl[di].name), "%s", "Controls");
    defControl[di].state =1;
    defControl[di].pdata = nullptr;
    defControl[di].other[0] = '\0';
    defControl[di].otherint =0;
    di++;

    auto addButton = [&](const char *label)
    {
        std::memset(&defControl[di],0, sizeof(mjuiDef));
        defControl[di].type = mjITEM_BUTTON;
        std::snprintf(defControl[di].name, sizeof(defControl[di].name), "%s", label);
        defControl[di].state =2;
        defControl[di].pdata = nullptr;
        defControl[di].other[0] = '\0';
        defControl[di].otherint =0;
        di++;
    };

    addButton(kBtnSpawnCube);
    addButton(kBtnStartPhysics);
    addButton(kBtnResetChain);
    addButton(kBtnIncreaseGap);
    addButton(kBtnDecreaseGap);
    addButton(kBtnSaveChain);
    addButton(kBtnLoadChain);
    addButton(kBtnPrintDirections);

    // terminator
    std::memset(&defControl[di],0, sizeof(mjuiDef));
    defControl[di].type = mjITEM_END;

    mjui_add(&ui, defControl);
    mjui_resize(&ui, &con);
    mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);

    // spawn a root cube on startup so the user doesn't have to
    spawnCube();

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;
        if (g_physicsEnabled)
        {
            while (d->time - simstart <1.0 /60.0)
            {
                mj_step(m, d);
            }
        }
        else
        {
            mj_kinematics(m, d);
            mj_collision(m, d);
        }

        mjrRect viewport = {0,0,0,0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        ui.rectid =0;
        uistate.nrect =1;
        int uiw = ui.width;
        if (uiw < g_uiWidth)
        {
            uiw = g_uiWidth;
        }
        uistate.rect[0] = {viewport.width - uiw,0, uiw, viewport.height};

        mjui_update(-1, -1, &ui, &uistate, &con);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // draw gap ratio in the upper-left corner as simple text overlay
        char gapText[64];
        std::snprintf(gapText, sizeof(gapText), "gap ratio: %.3f", g_gapRatio);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, gapText, NULL, &con);

        // draw save/load prompt when in file IO mode - move it to TOPLEFT to avoid UI overlap
        if (g_fileIoMode !=0)
        {
            char ioText[320];
            const char *mode = (g_fileIoMode ==1 ? "Save" : "Load");
            std::snprintf(ioText, sizeof(ioText), "%s filename: %s (Enter=OK, Esc=Cancel)", mode, g_ioFilename);
            // move to TOPLEFT so it's not behind the right-side UI
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, ioText, NULL, &con);
        }

        mjui_render(&ui, &uistate, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    if (spec)
    {
        mj_deleteSpec(spec);
        spec = nullptr;
    }

#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return EXIT_SUCCESS;
}
