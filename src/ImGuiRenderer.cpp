#include "ImGuiRenderer.h"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include <dxgi.h>
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg)
    {
    case WM_SIZE:
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU)
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProc(hWnd, msg, wParam, lParam);
}

ImGuiRenderer::ImGuiRenderer(int width, int height)
    : window_width(width), window_height(height), hwnd(nullptr),
      d3d_device(nullptr), d3d_device_context(nullptr),
      swap_chain(nullptr), render_target_view(nullptr)
{
}

ImGuiRenderer::~ImGuiRenderer()
{
    cleanup();
}

bool ImGuiRenderer::initialize()
{
    // Create window
    const wchar_t* class_name = L"MPC_Double_Pendulum";
    WNDCLASSW wc = { 0 };
    wc.style = CS_CLASSDC;
    wc.lpfnWndProc = WndProc;
    wc.cbClsExtra = 0L;
    wc.cbWndExtra = 0L;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.hIcon = nullptr;
    wc.hCursor = nullptr;
    wc.hbrBackground = nullptr;
    wc.lpszMenuName = nullptr;
    wc.lpszClassName = class_name;

    if (!RegisterClassW(&wc))
        return false;

    // hwnd = CreateWindowW(class_name, L"MPC Double Pendulum Balancer",
    //     WS_OVERLAPPEDWINDOW, 100, 100, window_width, window_height,
    //     nullptr, nullptr, wc.hInstance, nullptr);

    hwnd = CreateWindowEx(WS_EX_LAYERED | WS_EX_TOPMOST | WS_EX_NOACTIVATE, "MPC_Double_Pendulum", NULL,
        WS_POPUP, 0, 0, window_width, window_height,
        nullptr, nullptr, wc.hInstance, nullptr);
    SetLayeredWindowAttributes(hwnd, RGB(0,0,0), 0, ULW_COLORKEY);

    if (!hwnd)
        // std::cout << "Failed to create window" << std::endl;
        // std::cout << "Error code: " << GetLastError() << std::endl;
        return false;

    // Setup device and swap chain
    if (!createDeviceD3D())
    {
        cleanup();
        return false;
    }

    // Show window
    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    ImGui::StyleColorsDark();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(d3d_device, d3d_device_context);

    return true;
}

bool ImGuiRenderer::createDeviceD3D()
{
    D3D_FEATURE_LEVEL feature_level = D3D_FEATURE_LEVEL_11_0;
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = window_width;
    sd.BufferDesc.Height = window_height;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hwnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT create_device_flags = 0;
#ifdef _DEBUG
    create_device_flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
        create_device_flags, &feature_level, 1, D3D11_SDK_VERSION, &sd,
        &swap_chain, &d3d_device, nullptr, &d3d_device_context);

    if (res != S_OK)
        return false;

    ID3D11Texture2D* back_buffer = nullptr;
    swap_chain->GetBuffer(0, IID_PPV_ARGS(&back_buffer));
    if (back_buffer)
    {
        d3d_device->CreateRenderTargetView(back_buffer, nullptr, &render_target_view);
        back_buffer->Release();
    }

    return render_target_view != nullptr;
}

void ImGuiRenderer::cleanupDeviceD3D()
{
    if (render_target_view) { render_target_view->Release(); render_target_view = nullptr; }
    if (swap_chain) { swap_chain->Release(); swap_chain = nullptr; }
    if (d3d_device_context) { d3d_device_context->Release(); d3d_device_context = nullptr; }
    if (d3d_device) { d3d_device->Release(); d3d_device = nullptr; }
}

bool ImGuiRenderer::isRunning() const
{
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0U, 0U, PM_REMOVE))
    {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
        if (msg.message == WM_QUIT)
            return false;
    }
    return true;
}

void ImGuiRenderer::beginFrame()
{
    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiRenderer::endFrame()
{
    ImGui::Render();

    // const float clear_color_with_alpha[4] = { 0.45f, 0.55f, 0.60f, 1.00f };
    const float clear_color_with_alpha[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    d3d_device_context->OMSetRenderTargets(1, &render_target_view, nullptr);
    d3d_device_context->ClearRenderTargetView(render_target_view, clear_color_with_alpha);

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    swap_chain->Present(1, 0);
}

void ImGuiRenderer::render(DoublePendulum* pendulum, double control_torque, double mpc_cost, double time)
{
    beginFrame();
    static bool app_open = true;


    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(window_width - 20, window_height - 20), ImGuiCond_FirstUseEver);

    ImGui::Begin("Double Pendulum MPC Balancer", &app_open, ImGuiWindowFlags_AlwaysAutoResize);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    // ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    // canvas_size.y = 0.5*canvas_size.y; // Use half heght for canvas
    ImVec2 canvas_size(window_width/3, window_height/3);

    if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
    if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;

    // Draw canvas background
    draw_list->AddRectFilled(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        IM_COL32(50, 50, 50, 255)
    );

    draw_list->AddRect(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        IM_COL32(255, 255, 255, 255)
    );

    // Calculate center and scale
    ImVec2 center(canvas_pos.x + canvas_size.x / 2.0f, canvas_pos.y + canvas_size.y / 2.0f);
    float min_dim = canvas_size.x < canvas_size.y ? canvas_size.x : canvas_size.y;
    float scale = min_dim / 6.0f;

    // Get pendulum state
    State state = pendulum->getState();

    // Draw pivot point
    draw_list->AddCircleFilled(center, 6.0f, IM_COL32(255, 255, 0, 255));

    // Get joint positions
    double x1 = pendulum->getUpperJointX() * scale;
    double y1 = pendulum->getUpperJointY() * scale;
    double x2 = pendulum->getLowerJointX() * scale;
    double y2 = pendulum->getLowerJointY() * scale;

    // Draw first arm (upper - free)
    ImVec2 joint1(center.x + x1, center.y + y1);
    draw_list->AddLine(center, joint1, IM_COL32(0, 200, 100, 255), 4.0f);
    draw_list->AddCircleFilled(joint1, 5.0f, IM_COL32(0, 200, 100, 255));

    // Draw second arm (lower - controlled)
    ImVec2 joint2(center.x + x2, center.y + y2);
    draw_list->AddLine(joint1, joint2, IM_COL32(255, 100, 100, 255), 4.0f);
    draw_list->AddCircleFilled(joint2, 5.0f, IM_COL32(255, 100, 100, 255));

    ImGui::Dummy(canvas_size);

    // Display information
    ImGui::Separator();
    ImGui::Text("Simulation Time: %.2f s", time);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.4f, 1.0f), "State Information:");
    ImGui::Text("Upper Arm Angle (theta1):  %7.4f rad  (%7.2f deg)", state.theta1, state.theta1 * 180.0 / M_PI);
    ImGui::Text("Upper Arm Velocity:        %7.4f rad/s", state.theta1_dot);
    ImGui::Text("Lower Arm Angle (theta2):  %7.4f rad  (%7.2f deg)", state.theta2, state.theta2 * 180.0 / M_PI);
    ImGui::Text("Lower Arm Velocity:        %7.4f rad/s", state.theta2_dot);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.8f, 0.2f, 0.2f, 1.0f), "Control Information:");
    ImGui::Text("Applied Torque:            %7.4f N·m", control_torque);
    ImGui::Text("MPC Cost Function:         %7.4f", mpc_cost);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.8f, 1.0f), "Legend:");
    ImGui::BulletText("Green arm: Upper joint (free)");
    ImGui::BulletText("Red arm:   Lower joint (controlled)");
    ImGui::BulletText("Yellow dot: Pivot point");

    ImGui::End();

    if (!app_open) {
        ::PostQuitMessage(0);
    }

    endFrame();
}

void ImGuiRenderer::cleanup()
{
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    cleanupDeviceD3D();

    if (hwnd)
    {
        DestroyWindow(hwnd);
        UnregisterClassW(L"MPC_Double_Pendulum", GetModuleHandle(nullptr));
    }
}
