#ifndef __LOGGER__
#define __LOGGER__

#include <cxxabi.h>
#include <elf.h>
#include <execinfo.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <signal.h>
#include <unistd.h>

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include <Eigen/Core>
#include <fmt/format.h>
#include <sstream>

namespace fmt {
    template<typename Derived>
    struct formatter<Eigen::MatrixBase<Derived>> {
        template<typename ParseContext>
        constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

        template<typename FormatContext>
        auto format(const Eigen::MatrixBase<Derived>& m, FormatContext& ctx) const {
            std::stringstream ss;
            ss << m.transpose().format(Eigen::IOFormat(4, 1, ", ", "; "));
            return fmt::format_to(ctx.out(), "{}", ss.str());
        }
    };
}

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))

#define NO_INTR(fn)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
    } while ((fn) < 0 && errno == EINTR)

static int AssertFail()
{
    abort();
    return 0; // Should not reach.
}

#define SAFE_ASSERT(expr) ((expr) ? 0 : AssertFail())

#if !defined(SIZEOF_VOID_P)
#    if defined(__LP64__)
#        define SIZEOF_VOID_P 8
#    else
#        define SIZEOF_VOID_P 4
#    endif
#endif

#ifndef ElfW
#    if SIZEOF_VOID_P == 4
#        define ElfW(type) Elf32_##type
#    elif SIZEOF_VOID_P == 8
#        define ElfW(type) Elf64_##type
#    endif
#endif

static const std::map<int, std::string> kFailureSignals = {
    {SIGSEGV, "SIGSEGV"}, {SIGILL, "SIGILL"}, {SIGFPE, "SIGFPE"}, {SIGABRT, "SIGABRT"}};

struct FileDescriptor
{
    const int fd_;
    explicit FileDescriptor(int fd)
        : fd_(fd)
    {}
    ~FileDescriptor()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
    }
    int get()
    {
        return fd_;
    }

private:
    FileDescriptor(const FileDescriptor &);
    void operator=(const FileDescriptor &);
};

static int GetStackTrace(void **result, int max_depth, int skip_count)
{
    static const int kStackLength = 64;
    void *stack[kStackLength];
    int size;

    size = backtrace(stack, kStackLength);
    skip_count++; // we want to skip the current frame as well
    int result_count = size - skip_count;
    if (result_count < 0)
    {
        result_count = 0;
    }
    if (result_count > max_depth)
    {
        result_count = max_depth;
    }
    for (int i = 0; i < result_count; i++)
    {
        result[i] = stack[i + skip_count];
    }

    return result_count;
}

static void InvokeDefaultSignalHandler(int signal_number)
{
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(sig_action));
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_handler = SIG_DFL;
    sigaction(signal_number, &sig_action, NULL);
    kill(getpid(), signal_number);
}

static ssize_t ReadFromOffset(const int fd, void *buf, const size_t count, const size_t offset)
{
    SAFE_ASSERT(fd >= 0);
    SAFE_ASSERT(count <= static_cast<size_t>(std::numeric_limits<ssize_t>::max()));
    char *buf0 = reinterpret_cast<char *>(buf);
    size_t num_bytes = 0;
    while (num_bytes < count)
    {
        ssize_t len;
        NO_INTR(len = pread(fd, buf0 + num_bytes, count - num_bytes, static_cast<off_t>(offset + num_bytes)));
        if (len < 0)
        { // There was an error other than EINTR.
            return -1;
        }
        if (len == 0)
        { // Reached EOF.
            break;
        }
        num_bytes += static_cast<size_t>(len);
    }
    SAFE_ASSERT(num_bytes <= count);
    return static_cast<ssize_t>(num_bytes);
}

// Try reading exactly "count" bytes from "offset" bytes in a file
// pointed by "fd" into the buffer starting at "buf" while handling
// short reads and EINTR.  On success, return true. Otherwise, return
// false.
static bool ReadFromOffsetExact(const int fd, void *buf, const size_t count, const size_t offset)
{
    ssize_t len = ReadFromOffset(fd, buf, count, offset);
    return static_cast<size_t>(len) == count;
}

// Returns elf_header.e_type if the file pointed by fd is an ELF binary.
static int FileGetElfType(const int fd)
{
    ElfW(Ehdr) elf_header;
    if (!ReadFromOffsetExact(fd, &elf_header, sizeof(elf_header), 0))
    {
        return -1;
    }
    if (memcmp(elf_header.e_ident, ELFMAG, SELFMAG) != 0)
    {
        return -1;
    }
    return elf_header.e_type;
}

// Helper class for reading lines from file.
//
// Note: we don't use ProcMapsIterator since the object is big (it has
// a 5k array member) and uses async-unsafe functions such as sscanf()
// and snprintf().
class LineReader
{
public:
    explicit LineReader(int fd, char *buf, size_t buf_len, size_t offset)
        : fd_(fd)
        , buf_(buf)
        , buf_len_(buf_len)
        , offset_(offset)
        , bol_(buf)
        , eol_(buf)
        , eod_(buf)
    {}

    // Read '\n'-terminated line from file.  On success, modify "bol"
    // and "eol", then return true.  Otherwise, return false.
    //
    // Note: if the last line doesn't end with '\n', the line will be
    // dropped.  It's an intentional behavior to make the code simple.
    bool ReadLine(const char **bol, const char **eol)
    {
        if (BufferIsEmpty())
        { // First time.
            const ssize_t num_bytes = ReadFromOffset(fd_, buf_, buf_len_, offset_);
            if (num_bytes <= 0)
            { // EOF or error.
                return false;
            }
            offset_ += static_cast<size_t>(num_bytes);
            eod_ = buf_ + num_bytes;
            bol_ = buf_;
        }
        else
        {
            bol_ = eol_ + 1;           // Advance to the next line in the buffer.
            SAFE_ASSERT(bol_ <= eod_); // "bol_" can point to "eod_".
            if (!HasCompleteLine())
            {
                const size_t incomplete_line_length = static_cast<size_t>(eod_ - bol_);
                // Move the trailing incomplete line to the beginning.
                memmove(buf_, bol_, incomplete_line_length);
                // Read text from file and append it.
                char *const append_pos = buf_ + incomplete_line_length;
                const size_t capacity_left = buf_len_ - incomplete_line_length;
                const ssize_t num_bytes = ReadFromOffset(fd_, append_pos, capacity_left, offset_);
                if (num_bytes <= 0)
                { // EOF or error.
                    return false;
                }
                offset_ += static_cast<size_t>(num_bytes);
                eod_ = append_pos + num_bytes;
                bol_ = buf_;
            }
        }
        eol_ = FindLineFeed();
        if (eol_ == NULL)
        { // '\n' not found.  Malformed line.
            return false;
        }
        *eol_ = '\0'; // Replace '\n' with '\0'.

        *bol = bol_;
        *eol = eol_;
        return true;
    }

    // Beginning of line.
    const char *bol()
    {
        return bol_;
    }

    // End of line.
    const char *eol()
    {
        return eol_;
    }

private:
    LineReader(const LineReader &);
    void operator=(const LineReader &);

    char *FindLineFeed()
    {
        return reinterpret_cast<char *>(memchr(bol_, '\n', static_cast<size_t>(eod_ - bol_)));
    }

    bool BufferIsEmpty()
    {
        return buf_ == eod_;
    }

    bool HasCompleteLine()
    {
        return !BufferIsEmpty() && FindLineFeed() != NULL;
    }

    const int fd_;
    char *const buf_;
    const size_t buf_len_;
    size_t offset_;
    char *bol_;
    char *eol_;
    const char *eod_; // End of data in "buf_".
};

static char *GetHex(const char *start, const char *end, uint64_t *hex)
{
    *hex = 0;
    const char *p;
    for (p = start; p < end; ++p)
    {
        int ch = *p;
        if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F') || (ch >= 'a' && ch <= 'f'))
        {
            *hex = (*hex << 4U) | (ch < 'A' ? static_cast<uint64_t>(ch - '0') : (ch & 0xF) + 9U);
        }
        else
        { // Encountered the first non-hex character.
            break;
        }
    }
    SAFE_ASSERT(p <= end);
    return const_cast<char *>(p);
}

static int OpenObjectFileContainingPcAndGetStartAddress(
    uint64_t pc, uint64_t &start_address, uint64_t &base_address, std::string &out_file)
{
    int object_fd;
    int maps_fd;
    NO_INTR(maps_fd = open("/proc/self/maps", O_RDONLY));
    FileDescriptor wrapped_maps_fd(maps_fd);
    if (wrapped_maps_fd.get() < 0)
    {
        return -1;
    }
    int mem_fd;
    NO_INTR(mem_fd = open("/proc/self/mem", O_RDONLY));
    FileDescriptor wrapped_mem_fd(mem_fd);
    if (wrapped_mem_fd.get() < 0)
    {
        return -1;
    }
    // Iterate over maps and look for the map containing the pc.  Then
    // look into the symbol tables inside.
    char buf[1024]; // Big enough for line of sane /proc/self/maps
    unsigned num_maps = 0;
    LineReader reader(wrapped_maps_fd.get(), buf, sizeof(buf), 0);
    while (true)
    {
        num_maps++;
        const char *cursor;
        const char *eol;
        if (!reader.ReadLine(&cursor, &eol))
        { // EOF or malformed line.
            return -1;
        }
        // Start parsing line in /proc/self/maps.  Here is an example:
        //
        // 08048000-0804c000 r-xp 00000000 08:01 2142121    /bin/cat
        //
        // We want start address (08048000), end address (0804c000), flags
        // (r-xp) and file name (/bin/cat).

        // Read start address.
        cursor = GetHex(cursor, eol, &start_address);
        if (cursor == eol || *cursor != '-')
        {
            return -1; // Malformed line.
        }
        // Skip '-'.
        ++cursor;

        // Read end address.
        uint64_t end_address;
        cursor = GetHex(cursor, eol, &end_address);
        if (cursor == eol || *cursor != ' ')
        {
            return -1; // Malformed line.
        }
        // Skip ' '.
        ++cursor;
        // Read flags.  Skip flags until we encounter a space or eol.
        const char *const flags_start = cursor;
        while (cursor < eol && *cursor != ' ')
        {
            ++cursor;
        }
        // We expect at least four letters for flags (ex. "r-xp").
        if (cursor == eol || cursor < flags_start + 4)
        {
            return -1; // Malformed line.
        }
        // Determine the base address by reading ELF headers in process memory.
        ElfW(Ehdr) ehdr;
        // Skip non-readable maps.
        if (flags_start[0] == 'r' && ReadFromOffsetExact(mem_fd, &ehdr, sizeof(ElfW(Ehdr)), start_address) &&
            memcmp(ehdr.e_ident, ELFMAG, SELFMAG) == 0)
        {
            switch (ehdr.e_type)
            {
            case ET_EXEC:
                base_address = 0;
                break;
            case ET_DYN:
                // Find the segment containing file offset 0. This will correspond
                // to the ELF header that we just read. Normally this will have
                // virtual address 0, but this is not guaranteed. We must subtract
                // the virtual address from the address where the ELF header was
                // mapped to get the base address.
                //
                // If we fail to find a segment for file offset 0, use the address
                // of the ELF header as the base address.
                base_address = start_address;
                for (unsigned i = 0; i != ehdr.e_phnum; ++i)
                {
                    ElfW(Phdr) phdr;
                    if (ReadFromOffsetExact(
                            mem_fd, &phdr, sizeof(phdr), start_address + ehdr.e_phoff + i * sizeof(phdr)) &&
                        phdr.p_type == PT_LOAD && phdr.p_offset == 0)
                    {
                        base_address = start_address - phdr.p_vaddr;
                        break;
                    }
                }
                break;
            default:
                // ET_REL or ET_CORE. These aren't directly executable, so they don't
                // affect the base address.
                break;
            }
        }

        // Check start and end addresses.
        if (!(start_address <= pc && pc < end_address))
        {
            continue; // We skip this map.  PC isn't in this map.
        }

        // Check flags.  We are only interested in "r*x" maps.
        if (flags_start[0] != 'r' || flags_start[2] != 'x')
        {
            continue; // We skip this map.
        }
        // Skip ' '.
        ++cursor;
        // Read file offset.
        uint64_t file_offset;
        cursor = GetHex(cursor, eol, &file_offset);
        if (cursor == eol || *cursor != ' ')
        {
            return -1; // Malformed line.
        }
        // Skip ' '.
        ++cursor;

        // Skip to file name.  "cursor" now points to dev.  We need to
        // skip at least two spaces for dev and inode.
        int num_spaces = 0;
        while (cursor < eol)
        {
            if (*cursor == ' ')
            {
                ++num_spaces;
            }
            else if (num_spaces >= 2)
            {
                // The first non-space character after skipping two spaces
                // is the beginning of the file name.
                break;
            }
            ++cursor;
        }
        if (cursor == eol)
        {
            return -1; // Malformed line.
        }

        // Finally, "cursor" now points to file name of our interest.
        NO_INTR(object_fd = open(cursor, O_RDONLY));
        if (object_fd < 0)
        {
            return -1;
        }
        out_file.append(cursor);

        return object_fd;
    }
}

static bool GetSectionHeaderByType(
    const int fd, ElfW(Half) sh_num, const size_t sh_offset, ElfW(Word) type, ElfW(Shdr) * out)
{
    // Read at most 16 section headers at a time to save read calls.
    ElfW(Shdr) buf[16];
    for (size_t i = 0; i < sh_num;)
    {
        const size_t num_bytes_left = (sh_num - i) * sizeof(buf[0]);
        const size_t num_bytes_to_read = (sizeof(buf) > num_bytes_left) ? num_bytes_left : sizeof(buf);
        const ssize_t len = ReadFromOffset(fd, buf, num_bytes_to_read, sh_offset + i * sizeof(buf[0]));
        if (len == -1)
        {
            return false;
        }
        SAFE_ASSERT(static_cast<size_t>(len) % sizeof(buf[0]) == 0);
        const size_t num_headers_in_buf = static_cast<size_t>(len) / sizeof(buf[0]);
        SAFE_ASSERT(num_headers_in_buf <= sizeof(buf) / sizeof(buf[0]));
        for (size_t j = 0; j < num_headers_in_buf; ++j)
        {
            if (buf[j].sh_type == type)
            {
                *out = buf[j];
                return true;
            }
        }
        i += num_headers_in_buf;
    }
    return false;
}

static bool FindSymbol(uint64_t pc,
    const int fd,
    char *out,
    size_t out_size,
    uint64_t symbol_offset,
    const ElfW(Shdr) * strtab,
    const ElfW(Shdr) * symtab)
{
    if (symtab == NULL)
    {
        return false;
    }
    const size_t num_symbols = symtab->sh_size / symtab->sh_entsize;
    for (unsigned i = 0; i < num_symbols;)
    {
        size_t offset = symtab->sh_offset + i * symtab->sh_entsize;

        // If we are reading Elf64_Sym's, we want to limit this array to
        // 32 elements (to keep stack consumption low), otherwise we can
        // have a 64 element Elf32_Sym array.
#if defined(__WORDSIZE) && __WORDSIZE == 64
        const size_t NUM_SYMBOLS = 32U;
#else
        const size_t NUM_SYMBOLS = 64U;
#endif

        // Read at most NUM_SYMBOLS symbols at once to save read() calls.
        ElfW(Sym) buf[NUM_SYMBOLS];
        size_t num_symbols_to_read = std::min(NUM_SYMBOLS, num_symbols - i);
        const ssize_t len = ReadFromOffset(fd, &buf, sizeof(buf[0]) * num_symbols_to_read, offset);
        SAFE_ASSERT(static_cast<size_t>(len) % sizeof(buf[0]) == 0);
        const size_t num_symbols_in_buf = static_cast<size_t>(len) / sizeof(buf[0]);
        SAFE_ASSERT(num_symbols_in_buf <= num_symbols_to_read);
        for (unsigned j = 0; j < num_symbols_in_buf; ++j)
        {
            const ElfW(Sym) &symbol = buf[j];
            uint64_t start_address = symbol.st_value;
            start_address += symbol_offset;
            uint64_t end_address = start_address + symbol.st_size;
            if (symbol.st_value != 0 && // Skip null value symbols.
                symbol.st_shndx != 0 && // Skip undefined symbols.
                start_address <= pc && pc < end_address)
            {
                ssize_t len1 = ReadFromOffset(fd, out, out_size, strtab->sh_offset + symbol.st_name);
                if (len1 <= 0 || memchr(out, '\0', out_size) == NULL)
                {
                    memset(out, 0, out_size);
                    return false;
                }
                return true; // Obtained the symbol name.
            }
        }
        i += num_symbols_in_buf;
    }
    return false;
}

static bool GetSymbolFromObjectFile(const int fd, uint64_t pc, char *out, size_t out_size, uint64_t base_address)
{
    // Read the ELF header.
    ElfW(Ehdr) elf_header;
    if (!ReadFromOffsetExact(fd, &elf_header, sizeof(elf_header), 0))
    {
        return false;
    }

    ElfW(Shdr) symtab, strtab;

    // Consult a regular symbol table first.
    if (GetSectionHeaderByType(fd, elf_header.e_shnum, elf_header.e_shoff, SHT_SYMTAB, &symtab))
    {
        if (!ReadFromOffsetExact(fd, &strtab, sizeof(strtab), elf_header.e_shoff + symtab.sh_link * sizeof(symtab)))
        {
            return false;
        }
        if (FindSymbol(pc, fd, out, out_size, base_address, &strtab, &symtab))
        {
            return true; // Found the symbol in a regular symbol table.
        }
    }

    // If the symbol is not found, then consult a dynamic symbol table.
    if (GetSectionHeaderByType(fd, elf_header.e_shnum, elf_header.e_shoff, SHT_DYNSYM, &symtab))
    {
        if (!ReadFromOffsetExact(fd, &strtab, sizeof(strtab), elf_header.e_shoff + symtab.sh_link * sizeof(symtab)))
        {
            return false;
        }
        if (FindSymbol(pc, fd, out, out_size, base_address, &strtab, &symtab))
        {
            return true; // Found the symbol in a dynamic symbol table.
        }
    }

    return false;
}

static bool SymbolizeAndDemangle(void *pc, std::string &str_out)
{
    uint64_t pc0 = reinterpret_cast<uintptr_t>(pc);
    uint64_t start_address = 0;
    uint64_t base_address = 0;
    int object_fd = -1;

    std::string out_file;
    object_fd = OpenObjectFileContainingPcAndGetStartAddress(pc0, start_address, base_address, out_file);
    str_out.append(out_file);
    str_out.append(" : ");
    FileDescriptor wrapped_object_fd(object_fd);
    if (object_fd < 0)
    {
        if (out_file.c_str()[1])
        {
            // The object file containing PC was determined successfully however the
            // object file was not opened successfully.  This is still considered
            // success because the object file name and offset are known and tools
            // like asan_symbolize.py can be used for the symbolization.
            str_out.append("UNKNOW");
            return true;
        }
        // Failed to determine the object file containing PC.  Bail out.
        return false;
    }
    int elf_type = FileGetElfType(wrapped_object_fd.get());
    if (elf_type == -1)
    {
        return false;
    }
    const size_t out_size = 2048;
    char out[out_size];
    out[0] = '\0';
    if (!GetSymbolFromObjectFile(wrapped_object_fd.get(), pc0, out, out_size, base_address))
    {
        // The object file containing PC was opened successfully however the
        // symbol was not found. The object may have been stripped. This is still
        // considered success because the object file name and offset are known
        // and tools like asan_symbolize.py can be used for the symbolization.
        str_out.append("UNKNOW");
        return true;
    }
    int status;
    char *funcname = abi::__cxa_demangle(out, nullptr, nullptr, &status);
    if (status == 0)
    {
        str_out.append(funcname);
        free(funcname);
    }
    else
    {
        str_out.append(out);
    }
    return true;
}

class Logger;

static void FailureSignalHandler(int signal_number, siginfo_t *signal_info, void *ucontext);

static void InstallFailureSignalHandler()
{
    // Build the sigaction struct.
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(sig_action));
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags |= SA_SIGINFO;
    sig_action.sa_sigaction = FailureSignalHandler;

    for (auto &i : kFailureSignals)
    {
        sigaction(i.first, &sig_action, NULL);
    }
}

static std::string kLogFileName = "";

class Logger
{
public:
    static Logger &GetInstance()
    {
        static Logger instance{};
        return instance;
    }

    void init(const std::string &logName, const std::string &file, size_t maxSize, int maxFiles)
    {
        if (dailySink_ || fileSink_)
        {
            return;
        }
        // 创建循环logger,最多保存maxFiles个文件,每个文件最大maxSize
        fileSink_ = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file, maxSize, maxFiles);
        consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::always);
        consoleSink_->set_level(spdlog::level::debug);
        fileSink_->set_level(spdlog::level::info);

        spdlog::sinks_init_list sinks = {consoleSink_, fileSink_};
        kLogFileName = file;
        init_sinks(logName, sinks);
    }

    void init(const std::string &logName, const std::string &file, int maxDays)
    {
        if (dailySink_ || fileSink_)
        {
            return;
        }
        dailySink_ = std::make_shared<spdlog::sinks::daily_file_sink_mt>(file, // 日志文件基名，如 "logs/mylog.log"
            0,
            0,      // 分割时间：每天 0 点
            false,  // truncate = false，不清空旧文件
            maxDays // 最多保留的文件数（天数）
        );
        consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::always);
        consoleSink_->set_level(spdlog::level::debug);
        dailySink_->set_level(spdlog::level::info);

        spdlog::sinks_init_list sinks = {consoleSink_, dailySink_};
        kLogFileName = file;
        init_sinks(logName, sinks);
    }

    void setLogLevel(std::string logLevel)
    {
        if (fileSink_)
        {
            fileSink_->set_level(spdlog::level::from_str(logLevel));
        }
        else if (dailySink_)
        {
            dailySink_->set_level(spdlog::level::from_str(logLevel));
        }
        consoleSink_->set_level(spdlog::level::from_str(logLevel));
        logger_->set_level(spdlog::level::from_str(logLevel));
    }

    // clang-format on
    std::shared_ptr<spdlog::logger> logger()
    {
        return logger_;
    }

    Logger(const Logger &) = delete;
    Logger &operator=(const Logger) = delete;

private:
    void init_sinks(const std::string &logName, const spdlog::sinks_init_list &sinks)
    {
        logger_ = std::make_shared<spdlog::logger>(logName, std::begin(sinks), std::end(sinks));
        logger_->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e][%l][%s:%#][%!]: %v%$");
        // info及以上日志立即将缓存写入文件
        logger_->flush_on(spdlog::level::info);
        InstallFailureSignalHandler();
    }

private:
    Logger() {}
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> fileSink_;
    std::shared_ptr<spdlog::sinks::daily_file_sink_mt> dailySink_;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> consoleSink_;
};

static void FailureSignalHandler(int signal_number, siginfo_t *signal_info, void *ucontext)
{
    (void)signal_info;
    (void)ucontext;
    void *stack[32];
    const int depth = GetStackTrace(stack, ARRAYSIZE(stack), 1);

    std::string symbols;
    time_t ts = time(NULL);
    symbols.append("\nAbort time : " + std::string(ctime(&ts)));
    symbols.append("********************** signal " + kFailureSignals.at(signal_number));
    symbols.append(" stack print: **********************\n");
    for (int i = 0; i < depth; ++i)
    {
        std::string symbol;
        if (SymbolizeAndDemangle(stack[i], symbol))
        {
            symbol.append("\n");
            symbols.append(symbol);
        }
    }
    symbols.append("********************** stack print end **********************\n");
    std::fstream file;
    file.open(kLogFileName, std::ios::out | std::ios::app);
    file.write(symbols.c_str(), symbols.size());
    file.close();
    InvokeDefaultSignalHandler(signal_number);
}

// 直接使用 spdlog 的宏，它会正确处理编译期字符串
#define KR_LOGGER_CALL(level, ...) \
    SPDLOG_LOGGER_CALL(Logger::GetInstance().logger().get(), level, __VA_ARGS__)

#define KR_LOG_INIT(...) Logger::GetInstance().init(__VA_ARGS__)
#define KR_SET_LOG_LEVEL(...) Logger::GetInstance().setLogLevel(__VA_ARGS__)

#define KR_TRACE(...) KR_LOGGER_CALL(spdlog::level::trace, __VA_ARGS__)
#define KR_DEBUG(...) KR_LOGGER_CALL(spdlog::level::debug, __VA_ARGS__)
#define KR_INFO(...)  KR_LOGGER_CALL(spdlog::level::info,  __VA_ARGS__)
#define KR_WARN(...)  KR_LOGGER_CALL(spdlog::level::warn,  __VA_ARGS__)
#define KR_ERROR(...) KR_LOGGER_CALL(spdlog::level::err,   __VA_ARGS__)
#define KR_CRITICAL(...) KR_LOGGER_CALL(spdlog::level::critical, __VA_ARGS__)


#endif