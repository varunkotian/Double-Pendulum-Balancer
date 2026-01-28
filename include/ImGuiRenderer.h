#ifndef IMGUI_RENDERER_H
#define IMGUI_RENDERER_H

#include <d3d11.h>
#include <memory>
#include "DoublePendulum.h"

class ImGuiRenderer
{
public:
    ImGuiRenderer(int width = 1280, int height = 720);
    ~ImGuiRenderer();
    
    bool initialize();
    bool isRunning() const;
    void beginFrame();
    void endFrame();
    void render(DoublePendulum* pendulum, double control_torque, double mpc_cost, double time);
    void cleanup();
    
private:
    int window_width, window_height;
    HWND hwnd;
    ID3D11Device* d3d_device;
    ID3D11DeviceContext* d3d_device_context;
    IDXGISwapChain* swap_chain;
    ID3D11RenderTargetView* render_target_view;
    
    bool createDeviceD3D();
    void cleanupDeviceD3D();
    void drawPendulum(double x1, double y1, double x2, double y2);
};

#endif // IMGUI_RENDERER_H
