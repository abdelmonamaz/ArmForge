#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>

// ── Crash handler (Windows / MSVC) ───────────────────────────────────────
// Prints a symbolicated stack trace to stderr on unhandled SEH exceptions
// (access violations, stack overflows, ...) — the kind of native crash that
// qDebug()/try-catch never sees. Needs the Debug PDB to resolve symbols.
#ifdef Q_OS_WIN
#include <Windows.h>
#include <DbgHelp.h>
#include <array>
#include <cstdio>
#pragma comment(lib, "dbghelp.lib")

namespace {
    LONG WINAPI crashHandler(EXCEPTION_POINTERS* info)
    {
        const HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_DEFERRED_LOADS);
        SymInitialize(process, nullptr, TRUE);

        // fprintf kept deliberately — we're inside the faulting process' SEH
        // filter, where std::print's formatting machinery would add allocation/
        // exception paths we can't trust the crashed state to survive.
        fprintf(stderr, "\n===== ArmForge crashed — exception 0x%08lX at 0x%p =====\n", // NOSONAR (S6494)
                info->ExceptionRecord->ExceptionCode,
                info->ExceptionRecord->ExceptionAddress);

        CONTEXT ctx = *info->ContextRecord;
        STACKFRAME64 frame{};
        frame.AddrPC.Offset    = ctx.Rip;
        frame.AddrPC.Mode      = AddrModeFlat;
        frame.AddrFrame.Offset = ctx.Rbp;
        frame.AddrFrame.Mode   = AddrModeFlat;
        frame.AddrStack.Offset = ctx.Rsp;
        frame.AddrStack.Mode   = AddrModeFlat;

        alignas(SYMBOL_INFO) std::array<unsigned char, sizeof(SYMBOL_INFO) + 256> symbolStorage{};
        // SYMBOL_INFO is a WinAPI variable-length struct whose trailing name
        // buffer requires placing it over a raw aligned byte buffer —
        // reinterpret_cast is the documented pattern, no safer option here.
        auto* symbol = reinterpret_cast<SYMBOL_INFO*>(symbolStorage.data()); // NOSONAR (S3630)
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen   = 255;

        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);

        for (int i = 0; i < 64; ++i) {
            if (const bool walked = StackWalk64(IMAGE_FILE_MACHINE_AMD64, process, GetCurrentThread(),
                                                 &frame, &ctx, nullptr,
                                                 SymFunctionTableAccess64, SymGetModuleBase64, nullptr);
                !walked || frame.AddrPC.Offset == 0) {
                break;
            }

            DWORD64 symDisp  = 0;
            DWORD   lineDisp = 0;
            const char* name = "<unknown>";
            if (SymFromAddr(process, frame.AddrPC.Offset, &symDisp, symbol)) {
                name = symbol->Name;
            }

            // see fprintf rationale above — same crash-handler constraint
            if (SymGetLineFromAddr64(process, frame.AddrPC.Offset, &lineDisp, &line)) {
                fprintf(stderr, "  #%-2d 0x%016llX  %s  (%s:%lu)\n", // NOSONAR (S6494)
                        i, frame.AddrPC.Offset, name, line.FileName, line.LineNumber);
            } else {
                fprintf(stderr, "  #%-2d 0x%016llX  %s\n", i, frame.AddrPC.Offset, name); // NOSONAR (S6494)
            }
        }
        fprintf(stderr, "==========================================================\n"); // NOSONAR (S6494)
        fflush(stderr);

        return EXCEPTION_EXECUTE_HANDLER;
    }
}
#endif

int main(int argc, char *argv[])
{
#ifdef Q_OS_WIN
    SetUnhandledExceptionFilter(crashHandler);
#endif

    QQuickStyle::setStyle("Basic");

    QGuiApplication app(argc, argv);
    // S2209: use class-level :: for static methods, not instance dot-access
    QGuiApplication::setApplicationName("ArmForge");
    QGuiApplication::setApplicationVersion("0.3");

    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("ArmForge", "Main");

    return QCoreApplication::exec();
}
