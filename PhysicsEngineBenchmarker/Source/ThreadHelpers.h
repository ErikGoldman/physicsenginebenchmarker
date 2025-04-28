#pragma once
#include <functional>
#include <future>
#include <mutex>

class ThreadHelpers
{
public:
    static long long parallel_for(unsigned int num_threads, unsigned int n, const std::function<void(unsigned int, unsigned int)>& work_lambda)
    {
        std::mutex mtx;
        std::condition_variable cv;
        bool start_work_flag = false; // Flag to signal threads to start

        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        unsigned int chunk_size = n / num_threads;
        unsigned int remainder = n % num_threads;
        unsigned int current_start_n = 0;

        for (unsigned int i = 0; i < num_threads; ++i) {
            unsigned int start_n = current_start_n;
            unsigned int end_n = start_n + chunk_size + (i < remainder ? 1 : 0); // Distribute remainder

            // Pass the range [start_n, end_n), references to synchronization objects,
            // and the lambda function to the thread worker.
            threads.emplace_back(thread_worker, i, start_n, end_n,
                                 std::ref(mtx), std::ref(cv), std::ref(start_work_flag),
                                 work_lambda); // Pass the lambda
            current_start_n = end_n;
        }

        // threads now waiting
        {
            // Acquire the lock to safely modify the flag
            std::lock_guard<std::mutex> lock(mtx);
            start_work_flag = true; // Set the flag to true
        }
        auto start_time = std::chrono::steady_clock::now();
        cv.notify_all();
        for (std::thread& t : threads) {
            if (t.joinable()) {
                t.join(); // Blocks until thread t finishes execution
            }
        }
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = end_time - start_time;
        auto elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_time);
        long long total_nanos = elapsed_nanoseconds.count();
        return total_nanos;
    }

    template <typename Function>
    static long long ParallelFor(unsigned int num_threads, unsigned int num_items, Function&& func)
    {
        unsigned int block_size = num_items / num_threads;

        std::vector<std::future<void>> futures;
        unsigned int block_start = 0;

        auto start_time = std::chrono::steady_clock::now();
        for (unsigned int i = 0; i < num_threads; ++i)
        {
            unsigned int block_end = block_start + block_size;
            if (i == num_threads - 1)
            {
                block_end = num_items; // last thread takes remainder
            }

            futures.push_back(std::async(std::launch::async, [block_start, block_end, &func]()
            {
                func(block_start, block_end);
            }));

            block_start = block_end;
        }

        for (auto& f : futures)
        {
            f.get(); // Wait for all
        }
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = end_time - start_time;
        auto elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_time);
        long long total_nanos = elapsed_nanoseconds.count();
        return total_nanos;
    }

private:
    // The function executed by each spawned thread.
    // It waits for a signal before executing the provided lambda
    // for a specific range of input values (start_n to end_n).
    static void thread_worker(int ,
                       unsigned int start_n,
                       unsigned int end_n,
                       std::mutex& mtx,
                       std::condition_variable& cv,
                       bool& start_work_flag,
                       const std::function<void(unsigned int, unsigned int)>& work_lambda) { // Accepts the lambda by const reference
        {
            // Acquire the lock to check the condition
            std::unique_lock<std::mutex> lock(mtx);

            // Wait until the start_work_flag is true.
            // The wait() function automatically releases the lock and blocks the thread.
            // When notified, it reacquires the lock and checks the predicate.
            cv.wait(lock, [&]{ return start_work_flag; });

        } // The lock is automatically released when unique_lock goes out of scope

        // Now that the flag is set and we've been notified, execute the work lambda
        // for the assigned range of values.
        work_lambda(start_n, end_n); // Execute the lambda with the current value of n
    }


};
