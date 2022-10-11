#include <vector>

#include "compressors/entropy.h"


size_t EntropyCompressorNoDict::compress(
    const std::vector<unsigned char>& input, 
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;
    
    const size_t num_symbols = 1 << 8;
    
    // define numerical limits
    const unsigned int top = 1 << 24;
    const unsigned int bottom = 1 << 16;
    const unsigned int max_range = 1 << 16;

    size_t input_size = input.size();

    // init output vector
    output.reserve (input_size / 2);

    // calculate frequency table
    unsigned int freq_hist[num_symbols + 1];
    memset (freq_hist, 0, sizeof(freq_hist));

    size_t read_pos = 0;
    while (read_pos < input_size)
    {
        unsigned char symbol = input[read_pos++];
        freq_hist[symbol + 1]++;
    }

    // convert to cumulative frequency table
    unsigned int cumu_freq[num_symbols + 1];
    cumu_freq[0] = 0;
    for (size_t f = 1; f < num_symbols + 1; f++)
    {
        cumu_freq[f] = cumu_freq[f - 1] + freq_hist[f];
        if (cumu_freq[f] <= cumu_freq[f - 1])
            cumu_freq[f] = cumu_freq[f - 1] + 1;
    }

    // rescale if numerical limits are reached
    while (cumu_freq[num_symbols] >= max_range)
    {
        for (size_t f = 1; f < num_symbols + 1; f++)
        {
            cumu_freq[f] /= 2;
            if (cumu_freq[f] <= cumu_freq[f - 1])
            cumu_freq[f] = cumu_freq[f - 1] + 1;
        }
    }

    // write the input size and cumulative frequency
    // table to the output stream
    output.resize (sizeof(size_t) + sizeof(cumu_freq));
    memcpy(output.data(), &input_size, sizeof(size_t));
    memcpy(output.data() + sizeof(size_t), cumu_freq, sizeof(cumu_freq));

    read_pos = 0;
    unsigned int low = 0;
    unsigned int range = -1;

    // start encoding
    while (read_pos < input_size)
    {
        // read symol
        unsigned char symbol = input[read_pos++];

        // map to range
        low += cumu_freq[symbol] * (range /= cumu_freq[num_symbols]);
        range *= cumu_freq[symbol + 1] - cumu_freq[symbol];

        // check range limits
        while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -static_cast<int>(low) & (bottom - 1)), 1)))
        {
            char outc = static_cast<char> (low >> 24);
            range <<= 8;
            low <<= 8;
            output.push_back (outc);
        }
    }

    // flush remaining data
    for (int i = 0; i < 4; i++)
    {
        char outc = static_cast<char> (low >> 24);
        output.push_back (outc);
        low <<= 8;
    }

    return (output.size());
}

size_t EntropyCompressorNoDict::decompress(
    const std::vector<unsigned char>& input,
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;

    const size_t num_symbols = 1 << 8;
    
    // define numerical limits
    const unsigned int top = 1 << 24;
    const unsigned int bottom = 1 << 16;

    size_t input_size = input.size();

    // init output vector
    output.reserve (input_size * 2);

    // read output_size and cumulative frequency table
    size_t output_size;
    unsigned int cumu_freq[num_symbols + 1];
    memcpy(&output_size, input.data(), sizeof(size_t));
    memcpy(cumu_freq, input.data() + sizeof(size_t), sizeof(cumu_freq));

    size_t read_pos = sizeof(size_t) + sizeof(cumu_freq);
    unsigned int code = 0;
    unsigned int low = 0;
    unsigned int range = -1;

    // init code
    for (int i = 0; i < 4; i++)
    {
        unsigned char ch = input[read_pos++];
        code = (code << 8) | ch;
    }
    // decoding
    while (output.size() < output_size)
    {
        // find the symbol
        unsigned char symbol = 0;
        unsigned char half_size = 256 / 2;

        unsigned int count = (code - low) / (range /= cumu_freq[256]);

        while (half_size > 0)
        {
            if (cumu_freq[symbol + half_size] <= count)
            {
                symbol += half_size;
            }
            half_size /= 2;
        }

        output.push_back(symbol);

        low += cumu_freq[symbol] * range;
        range *= cumu_freq[symbol + 1] - cumu_freq[symbol];

        // check the range limits
        while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -static_cast<int>(low) & (bottom - 1)), 1)))
        {
            unsigned char ch = input[read_pos++];
            code = code << 8 | ch;
            range <<= 8;
            low <<= 8;
        }
    }
    return output.size();
}

size_t EntropyCompressor::compress(
    const std::vector<int>& input, 
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;

    // define numerical limits
    const unsigned long top = static_cast<unsigned long> (1) << 56;
    const unsigned long bottom = static_cast<unsigned long> (1) << 48;
    const size_t maxRange = static_cast<unsigned long> (1) << 48;

    size_t input_size = input.size ();

    // init output vector
    output.reserve (input_size / 2);

    // statistics occurances of all symbols
    std::map<int, size_t> freq_hist;

    size_t read_pos = 0;
    while (read_pos < input_size)
    {
        int symbol = input[read_pos++];

        if (freq_hist.find(symbol) == freq_hist.end())
            freq_hist[symbol] = 1;
        else
            freq_hist[symbol] ++;
    }

    // convert to cumulative frequency table
    std::unordered_map<int, unsigned int> symbol_index;

    size_t num_symbols = freq_hist.size();
    int min_symbol = freq_hist.begin()->first;
    std::vector <size_t> cumu_freq (num_symbols + 1);
    cumu_freq[0] = 0;
    
    for (auto [idx, it] = std::tuple{0, freq_hist.begin()}; it != freq_hist.end(); idx++, it++)
    {
        auto& symbol = it->first;
        symbol_index[symbol] = idx;
        cumu_freq[idx + 1] = cumu_freq[idx] + freq_hist[symbol];
    }


    // rescale if numerical limits are reached
    while (cumu_freq.back() >= maxRange)
    {
        for (size_t i = 1; i < num_symbols + 1; i++)
        {
            cumu_freq[i] /= 2;
            if (cumu_freq[i] <= cumu_freq[i - 1])
            cumu_freq[i] = cumu_freq[i - 1] + 1;
        }
    }

    // calculate amount of bytes per frequency table entry
    unsigned char num_bytes_per_symbol = std::ceil(std::log2(freq_hist.rbegin()->first - min_symbol + 1) / 8.0);
    unsigned char num_bytes_per_freq = std::ceil(std::log2(cumu_freq.back() + 1) / 8.0);

    // write size of frequency table
    size_t write_pos = 0UL;
    output.resize(
        sizeof(size_t) +        // input size
        sizeof(int) +           // min symbol
        sizeof(int) +           // num_symbols
        sizeof(unsigned char) + // num_bytes_per_symbol
        sizeof(unsigned char) + // num_bytes_per_freq
        num_symbols * num_bytes_per_symbol + 
        num_symbols * num_bytes_per_freq
    );

    memcpy(output.data() + write_pos, &input_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, &min_symbol, sizeof(int));
    write_pos += sizeof(int);
    memcpy(output.data() + write_pos, &num_symbols, sizeof(int));
    write_pos += sizeof(int);
    memcpy(output.data() + write_pos, &num_bytes_per_symbol, 1);
    write_pos += 1;
    memcpy(output.data() + write_pos, &num_bytes_per_freq, 1);
    write_pos += 1;

    // write cumulative frequency table to output stream
    for (auto it = freq_hist.begin(); it != freq_hist.end(); it++)
    {
        int delta = it->first - min_symbol;
        memcpy(output.data() + write_pos, &delta, num_bytes_per_symbol);
        write_pos += num_bytes_per_symbol;
    }
    for (size_t i = 0; i < num_symbols; i++)
    {
        size_t cur_freq = cumu_freq[i + 1];
        memcpy(output.data() + write_pos, &cur_freq, num_bytes_per_freq);
        write_pos += num_bytes_per_freq;
    }

    read_pos = 0;
    unsigned long low = 0;
    unsigned long range = static_cast<unsigned long>(-1);

    // start encoding
    while (read_pos < input_size)
    {
        int symbol = input[read_pos++];

        // map to range
        low += cumu_freq[symbol_index[symbol]] * (range /= cumu_freq.back());
        range *= cumu_freq[symbol_index[symbol] + 1] - cumu_freq[symbol_index[symbol]];

        // check range limits
        while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
        {
            char out = static_cast<char> (low >> 56);
            range <<= 8;
            low <<= 8;
            output.push_back (out);
        }

    }

    // flush remaining data
    for (unsigned int i = 0; i < 8; i++)
    {
        char out = static_cast<char> (low >> 56);
        output.push_back (out);
        low <<= 8;
    }

    return (output.size());
}

size_t EntropyCompressor::decompress(
    const std::vector<unsigned char>& input, 
    std::vector<int>& output
) {
    output.clear();
    if (input.empty()) return 0UL;

    // define numerical limits
    const unsigned long top = static_cast<unsigned long> (1) << 56;
    const unsigned long bottom = static_cast<unsigned long> (1) << 48;

    size_t input_size = input.size ();

    // init output vector
    output.reserve (input_size * 2);

    size_t output_size;
    int min_symbol;
    unsigned int num_symbols;
    unsigned char num_bytes_per_symbol, num_bytes_per_freq;
    
    size_t read_pos = 0UL;
    memcpy(&output_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    memcpy(&min_symbol, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&num_symbols, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&num_bytes_per_symbol, input.data() + read_pos, sizeof(unsigned char));
    read_pos += sizeof(unsigned char);
    memcpy(&num_bytes_per_freq, input.data() + read_pos, sizeof(unsigned char));
    read_pos += sizeof(unsigned char);

    std::vector <size_t> cumu_freq (num_symbols + 1);
    cumu_freq[0] = 0UL;
    std::vector <int> symbols (num_symbols);

    for (unsigned int i = 0; i < num_symbols; i++)
    {
        memcpy(&symbols[i], input.data() + read_pos, num_bytes_per_symbol);
        symbols[i] += min_symbol;
        read_pos += num_bytes_per_symbol;
    }
    for (unsigned int i = 0; i < num_symbols; i++)
    {
        memcpy(&cumu_freq[i + 1], input.data() + read_pos, num_bytes_per_freq);
        read_pos += num_bytes_per_freq;
    }

    unsigned long code = 0;
    unsigned long low = 0;
    unsigned long range = static_cast<unsigned long>(-1);

    // init code
    for (int i = 0; i < 8; i++)
    {
        unsigned char ch = input[read_pos++];
        code = (code << 8) | ch;
    }
    while (output.size() < output_size)
    {
        // find the symbol
        unsigned int symbol_idx = 0;
        unsigned int half_size = std::pow(2, std::ceil(std::log2(num_symbols))) / 2;

        unsigned long count = (code - low) / (range /= cumu_freq.back());

        while (half_size > 0)
        {
            if (symbol_idx + half_size < num_symbols && cumu_freq[symbol_idx + half_size] <= count)
            {
                symbol_idx += half_size;
            }
            half_size /= 2;
        }

        output.push_back(symbols[symbol_idx]);

        low += cumu_freq[symbol_idx] * range;
        range *= cumu_freq[symbol_idx + 1] - cumu_freq[symbol_idx];

        // check the range limits
        while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -static_cast<long>(low) & (bottom - 1)), 1)))
        {
            unsigned char ch = input[read_pos++];
            code = code << 8 | ch;
            range <<= 8;
            low <<= 8;
        }
    }
    return output.size();
}
