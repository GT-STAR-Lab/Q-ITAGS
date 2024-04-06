/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Global
#include <list>
#include <set>
#include <vector>
// External
#include <gtest/gtest.h>
// Project
#include <grstapse/common/utilities/dereference_container_wrapper.hpp>

namespace grstapse::unittests
{
    //! Simple struct for testing on a non-primitive type
    struct IntWrapper
    {
        int value;
    };

    TEST(DereferenceContainerWrapper, is_container)
    {
        // region List
        {
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::list>::iterator>);
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::list>::const_iterator>);
            static_assert(std::bidirectional_iterator<DereferenceContainerWrapper<int, std::list>::iterator>);
            static_assert(std::bidirectional_iterator<DereferenceContainerWrapper<int, std::list>::const_iterator>);
            static_assert(std_ext::Container<DereferenceContainerWrapper<int, std::list>>);
        }
        // endregion
        // region Vector
        {
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::vector>::iterator>);
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::vector>::const_iterator>);
            static_assert(std::bidirectional_iterator<DereferenceContainerWrapper<int, std::vector>::iterator>);
            static_assert(std::bidirectional_iterator<DereferenceContainerWrapper<int, std::vector>::const_iterator>);
            static_assert(std::random_access_iterator<DereferenceContainerWrapper<int, std::vector>::iterator>);
            static_assert(std::random_access_iterator<DereferenceContainerWrapper<int, std::vector>::const_iterator>);
            static_assert(std_ext::Container<DereferenceContainerWrapper<int, std::vector>>);
        }
        // endregion
        // region Set
        {
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::set>::iterator>);
            static_assert(std::forward_iterator<DereferenceContainerWrapper<int, std::set>::const_iterator>);
            static_assert(std_ext::Container<DereferenceContainerWrapper<int, std::set>>);
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, front)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                IntWrapper& f = wrapper.front();
                EXPECT_EQ(f.value, a.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                const IntWrapper& f = const_wrapper.front();
                EXPECT_EQ(f.value, a.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                IntWrapper& f = wrapper.front();
                EXPECT_EQ(f.value, a.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                const IntWrapper& f = const_wrapper.front();
                EXPECT_EQ(f.value, a.value);
            }
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, back)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                IntWrapper& f = wrapper.back();
                EXPECT_EQ(f.value, d.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                const IntWrapper& f = const_wrapper.back();
                EXPECT_EQ(f.value, d.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                IntWrapper& f = wrapper.back();
                EXPECT_EQ(f.value, d.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                const IntWrapper& f = const_wrapper.back();
                EXPECT_EQ(f.value, d.value);
            }
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, begin)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = wrapper.begin();
                EXPECT_EQ(iter->value, a.value);
            }
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, end)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = --wrapper.end();
                EXPECT_EQ(iter->value, d.value);
            }
        }
    }

    TEST(DereferenceContainerWrapper, range)
    {
        IntWrapper a                         = {1};
        IntWrapper b                         = {2};
        IntWrapper c                         = {3};
        IntWrapper d                         = {4};
        std::vector<IntWrapper> ground_truth = {a, b, c, d};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                std::size_t i = 0;
                for(IntWrapper& iw: wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                std::size_t i = 0;
                for(const IntWrapper& iw: const_wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                std::size_t i = 0;
                for(IntWrapper& iw: wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                std::size_t i = 0;
                for(const IntWrapper& iw: const_wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                std::size_t i = 0;
                for(IntWrapper& iw: wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }

            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                std::size_t i = 0;
                for(const IntWrapper& iw: const_wrapper)
                {
                    EXPECT_EQ(iw.value, ground_truth[i].value);
                    ++i;
                }
            }
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, pre_increment)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter   = wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter   = wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter   = wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.begin();
                auto& iter2 = ++iter;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, b.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, post_increment)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter  = wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter  = wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter  = wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.begin();
                auto iter2 = iter++;
                EXPECT_EQ(iter->value, b.value);
                EXPECT_EQ(iter2->value, a.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, self_increment)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter = wrapper.begin();
                iter += 2;
                EXPECT_EQ(iter->value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = const_wrapper.begin();
                iter += 2;
                EXPECT_EQ(iter->value, c.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, increment)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter  = wrapper.begin();
                auto iter2 = iter + 2;
                EXPECT_EQ(iter->value, a.value);
                EXPECT_EQ(iter2->value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.begin();
                auto iter2 = iter + 2;
                EXPECT_EQ(iter->value, a.value);
                EXPECT_EQ(iter2->value, c.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, other_increment)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter  = wrapper.begin();
                auto iter2 = 2 + iter;
                EXPECT_EQ(iter->value, a.value);
                EXPECT_EQ(iter2->value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.begin();
                auto iter2 = 2 + iter;
                EXPECT_EQ(iter->value, a.value);
                EXPECT_EQ(iter2->value, c.value);
            }
        }
        // endregion
    }

    TEST(DereferenceContainerWrapper, pre_decrement)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter   = wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter   = wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter   = wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter   = const_wrapper.end();
                auto& iter2 = --iter;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2->value, d.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, post_decrement)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region List
        {
            std::list<IntWrapper*> l = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&l);
            {
                auto iter  = wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, wrapper.end());
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, const_wrapper.end());
            }
        }
        // endregion
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter  = wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, wrapper.end());
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, const_wrapper.end());
            }
        }
        // endregion
        // region Set
        {
            std::set<IntWrapper*> s = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&s);
            {
                auto iter  = wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, wrapper.end());
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.end();
                auto iter2 = iter--;
                EXPECT_EQ(iter->value, d.value);
                EXPECT_EQ(iter2, const_wrapper.end());
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, self_decrement)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter = wrapper.end();
                iter -= 2;
                EXPECT_EQ(iter->value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = const_wrapper.end();
                iter -= 2;
                EXPECT_EQ(iter->value, c.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, decrement)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter  = wrapper.end();
                auto iter2 = iter - 2;
                EXPECT_EQ(iter, wrapper.end());
                EXPECT_EQ(iter2->value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter  = const_wrapper.end();
                auto iter2 = iter - 2;
                EXPECT_EQ(iter, const_wrapper.end());
                EXPECT_EQ(iter2->value, c.value);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, difference)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto diff = wrapper.end() - wrapper.begin();
                EXPECT_EQ(diff, 4);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto diff = const_wrapper.end() - const_wrapper.begin();
                EXPECT_EQ(diff, 4);
            }
        }
        // endregion
    }
    TEST(DereferenceContainerWrapper, brackets)
    {
        IntWrapper a = {1};
        IntWrapper b = {2};
        IntWrapper c = {3};
        IntWrapper d = {4};
        // region Vector
        {
            std::vector<IntWrapper*> v = {&a, &b, &c, &d};
            DereferenceContainerWrapper wrapper(&v);
            {
                auto iter = wrapper.begin();
                auto& r   = iter[2];
                EXPECT_EQ(r.value, c.value);
            }
            const DereferenceContainerWrapper const_wrapper = wrapper;
            {
                auto iter = const_wrapper.begin();
                auto& r   = iter[2];
                EXPECT_EQ(r.value, c.value);
            }
        }
        // endregion
    }
}  // namespace grstapse::unittests