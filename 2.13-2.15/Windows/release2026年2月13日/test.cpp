#include <iostream>
#include <thread>
#include <barrier>
#include <vector>

int main() {
    const int thread_count = 4;

    // barrier 的回调函数：当所有线程到达 barrier 时执行一次
    std::barrier sync_point(thread_count, [](){
        std::cout << "All threads reached the barrier. Continue...\n";
    });

    auto worker = [&](int id) {
        std::cout << "Thread " << id << " working...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100 * id));

        // 到达同步点
        sync_point.arrive_and_wait();

        std::cout << "Thread " << id << " passed the barrier.\n";
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < thread_count; ++i)
        threads.emplace_back(worker, i);

    for (auto& t : threads)
        t.join();

    return 0;
}
